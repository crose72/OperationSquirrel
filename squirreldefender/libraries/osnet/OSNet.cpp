#include "OSNet.h"
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <algorithm>
#include <cmath>

OSNet::OSNet(const std::string &trtModelPath, const Config &config)
    : mFeatureDim(config.featureDim),
      mDoL2Normalize(config.l2Normalize)
{
    if (!trtModelPath.empty())
    {
        mEnginePath = trtModelPath;
        mEngine = std::make_unique<TRTEngine<float>>(trtModelPath);

        bool engineLoaded = mEngine->loadNetwork(trtModelPath, mMean, mStd, mNormalize);
        if (!engineLoaded)
        {
            const std::string msg = "Error: Unable to load TensorRT engine from " + trtModelPath;
            spdlog::error(msg);
        }
    }
    else
    {
        const std::string msg = "No TensorRT engine path provided.";
        spdlog::error(msg);
    }

    // ---- Input dims ----
    const std::vector<nvinfer1::Dims> inputDims = mEngine->getInputDims();
    if (inputDims.empty())
    {
        spdlog::error("OSNet: no input bindings found.");
    }
    else
    {
        if (inputDims[0].nbDims == 4)
        {
            // [N, C, H, W]
            mEngineInputHeight = inputDims[0].d[2];
            mEngineInputWidth = inputDims[0].d[3];
        }
        else if (inputDims[0].nbDims == 3)
        {
            // [C, H, W] (N omitted)
            mEngineInputHeight = inputDims[0].d[1];
            mEngineInputWidth = inputDims[0].d[2];
        }
        else
        {
            const std::string msg = "Unsupported input tensor for OSNet, nbDims: " + std::to_string(inputDims[0].nbDims);
            spdlog::error(msg);
        }
    }

    // ---- Output dims ----
    const std::vector<nvinfer1::Dims> outputDims = mEngine->getOutputDims();
    if (outputDims.empty())
    {
        spdlog::error("OSNet: no output bindings found.");
    }
    else
    {
        // OSNet typically: [N, feat_dim] or [feat_dim]
        const auto &od = outputDims[0];
        if (od.nbDims == 2)
        {
            // [N, D]
            mEngineBatchSize = od.d[0];
            // od.d[1] can be -1 for dynamic, else feature dim
            if (od.d[1] > 0)
                mFeatureDim = od.d[1];
        }
        else if (od.nbDims == 1)
        {
            // [D] => batch=1 implied
            mEngineBatchSize = 1;
            if (od.d[0] > 0)
                mFeatureDim = od.d[0];
        }
        else
        {
            const std::string msg = "Unsupported output tensor for OSNet, nbDims: " + std::to_string(od.nbDims);
            spdlog::error(msg);
        }
    }
}

std::vector<std::vector<cv::cuda::GpuMat>> OSNet::preprocess(const cv::cuda::GpuMat &gpuImg)
{
    std::vector<cv::cuda::GpuMat> inputImg{gpuImg};
    return preprocess(inputImg);
}

std::vector<std::vector<cv::cuda::GpuMat>> OSNet::preprocess(const std::vector<cv::cuda::GpuMat> &gpuImgs)
{
    mActualBatchSize = static_cast<int>(gpuImgs.size());

    std::vector<cv::cuda::GpuMat> resizedImgs;
    resizedImgs.reserve(gpuImgs.size());

    for (const auto &gpuImgBGR : gpuImgs)
    {
        // Convert BGR -> RGB (OSNet typically trained on RGB)
        cv::cuda::GpuMat rgbMat;
        cv::cuda::cvtColor(gpuImgBGR, rgbMat, cv::COLOR_BGR2RGB);

        // Strict resize to model input (no letterbox; ReID expects tight crops)
        cv::cuda::GpuMat resized;
        if (rgbMat.rows != mEngineInputHeight || rgbMat.cols != mEngineInputWidth)
        {
            cv::cuda::resize(rgbMat, resized, cv::Size((int)mEngineInputWidth, (int)mEngineInputHeight), 0, 0, cv::INTER_LINEAR);
        }
        else
        {
            resized = rgbMat;
        }

        // DO NOT manually scale/normalize here—TRTEngine will do [0..1] + mean/std.
        resizedImgs.push_back(resized);
    }

    // Engine expects vector<bindings> where each binding is a vector<batchImages>.
    // Single input binding => wrap resizedImgs as [[img1..imgN]]
    std::vector<std::vector<cv::cuda::GpuMat>> engineInputBatch;
    engineInputBatch.push_back(std::move(resizedImgs));
    return engineInputBatch;
}

std::vector<float> OSNet::extractFeatures(const cv::Mat &inputImageBGR)
{
    cv::cuda::GpuMat gpuImg;
    gpuImg.upload(inputImageBGR);
    return extractFeatures(gpuImg);
}

std::vector<float> OSNet::extractFeatures(const cv::cuda::GpuMat &inputImageBGR)
{
    std::vector<cv::cuda::GpuMat> batch{inputImageBGR};
    auto feats = extractFeatures(batch);
    if (!feats.empty())
        return feats[0];
    return {};
}

std::vector<std::vector<float>> OSNet::extractFeatures(const std::vector<cv::Mat> &batchImgsBGR)
{
    std::vector<cv::cuda::GpuMat> gpuBatch;
    gpuBatch.reserve(batchImgsBGR.size());
    for (const auto &img : batchImgsBGR)
    {
        cv::cuda::GpuMat g;
        g.upload(img);
        gpuBatch.push_back(std::move(g));
    }
    return extractFeatures(gpuBatch);
}

std::vector<std::vector<float>> OSNet::extractFeatures(const std::vector<cv::cuda::GpuMat> &batchImgsBGR)
{
#ifdef ENABLE_BENCHMARKS
    static int numIts = 1;
    preciseStopwatch s1;
#endif

    // Preprocess into engine input shape/layout
    const std::vector<std::vector<cv::cuda::GpuMat>> engineInputBatch = preprocess(batchImgsBGR);

#ifdef ENABLE_BENCHMARKS
    static long long t1 = 0;
    t1 += s1.elapsedTime<long long, std::chrono::microseconds>();
    spdlog::info("OSNet Avg Preprocess time: {:.3f} ms", (t1 / numIts) / 1000.f);
    preciseStopwatch s2;
#endif

    // Run inference
    // featureVectors layout (matching your YOLO engine usage):
    //   vector<batch> -> vector<outputs> -> vector<float>(flat tensor)
    std::vector<std::vector<std::vector<float>>> featureVectors;
    bool ok = mEngine->runInference(engineInputBatch, featureVectors);
    if (!ok)
    {
        spdlog::error("OSNet: Failed to run inference");
        return {};
    }

#ifdef ENABLE_BENCHMARKS
    static long long t2 = 0;
    t2 += s2.elapsedTime<long long, std::chrono::microseconds>();
    spdlog::info("OSNet Avg Inference time: {:.3f} ms", (t2 / numIts) / 1000.f);
    preciseStopwatch s3;
#endif

    // Expect ONE output tensor per image: embedding
    // featureVectors[batchIdx][0] = flat embedding
    if ((int)featureVectors.size() != mActualBatchSize)
    {
        spdlog::warn("OSNet: unexpected batch dim in outputs. got={}, expected={}", featureVectors.size(), mActualBatchSize);
    }

    std::vector<std::vector<float>> embeddings;
    embeddings.reserve(featureVectors.size());

    for (size_t b = 0; b < featureVectors.size(); ++b)
    {
        if (featureVectors[b].empty())
        {
            spdlog::error("OSNet: missing output tensor at batch index {}", b);
            embeddings.push_back({});
            continue;
        }

        auto &flatFeat = featureVectors[b][0];

        // If the engine returned [D * something], trim if needed:
        if ((int)flatFeat.size() != mFeatureDim)
        {
            if ((int)flatFeat.size() > mFeatureDim)
            {
                flatFeat.resize(mFeatureDim);
            }
            else
            {
                spdlog::warn("OSNet: embedding dim {} < expected {}; zero-padding.", flatFeat.size(), mFeatureDim);
                flatFeat.resize(mFeatureDim, 0.f);
            }
        }

        if (mDoL2Normalize)
        {
            l2Normalize(flatFeat);
        }

        embeddings.push_back(std::move(flatFeat));
    }

#ifdef ENABLE_BENCHMARKS
    static long long t3 = 0;
    t3 += s3.elapsedTime<long long, std::chrono::microseconds>();
    spdlog::info("OSNet Avg Postprocess time: {:.3f} ms", (t3 / numIts++) / 1000.f);
#endif

    return embeddings;
}

bool OSNet::l2Normalize(std::vector<float> &feat)
{
    double acc = 0.0;
    for (float v : feat)
    {
        acc += (double)v * (double)v;
    }
    if (acc <= 0.0)
    {
        return false;
    }

    const float inv = 1.0f / static_cast<float>(std::sqrt(acc));

    for (auto &v : feat)
    {
        v *= inv;
    }
    return true;
}
