#ifndef OSNET_H
#define OSNET_H

#include "TRTEngine.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <string>
#include <array>
#include <vector>

class OSNet
{
public:
    // Config for the ReID extractor
    struct Config
    {
        // Embedding dimension (commonly 512 for OSNet)
        int featureDim = 512;
        // Whether to L2-normalize embeddings before returning
        bool l2Normalize = true;
    };

    // Builds/loads the TensorRT engine and reads IO shapes
    OSNet(const std::string &trtModelPath, const Config &config);
    std::vector<float> extractFeatures(const cv::Mat &inputImageBGR);
    std::vector<float> extractFeatures(const cv::cuda::GpuMat &inputImageBGR);
    std::vector<std::vector<float>> extractFeatures(const std::vector<cv::Mat> &batchImgsBGR);
    std::vector<std::vector<float>> extractFeatures(const std::vector<cv::cuda::GpuMat> &batchImgsBGR);
    void printEngineInfo(void) { mEngine->printEngineInfo(); };

private:
    // Overloaded function for single input preprocessing
    std::vector<std::vector<cv::cuda::GpuMat>> preprocess(const cv::cuda::GpuMat &gpuImg);

    // Overloaded function for batch input preprocessing
    std::vector<std::vector<cv::cuda::GpuMat>> preprocess(const std::vector<cv::cuda::GpuMat> &gpuImgs);

    // L2 normalize a single embedding in-place; returns false if norm==0
    static bool l2Normalize(std::vector<float> &feat);

    // Engine instance
    std::string mEnginePath;
    std::unique_ptr<TRTEngine<float>> mEngine;

    // Engine input/output parameters
    int64_t mEngineInputHeight = 256; // OSNet default H
    int64_t mEngineInputWidth = 128;  // OSNet default W
    int64_t mEngineBatchSize = 1;     // read from output dims if available
    int mFeatureDim = 512;            // from config

    // Mean/Std/Normalize flags (TRTEngine handles normalization)
    // OSNet typically uses ImageNet statistics on [0..1] inputs.
    std::array<float, 3> mMean = {0.485f, 0.456f, 0.406f};
    std::array<float, 3> mStd = {0.229f, 0.224f, 0.225f};
    bool mNormalize = true; // normalize to [0,1] then mean/std in engine

    // Runtime params
    int mActualBatchSize = 1;

    // Config flags
    bool mDoL2Normalize = true;
};

#endif // OSNET_H