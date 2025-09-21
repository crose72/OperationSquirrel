#include "YOLOv8.h"
#include <algorithm>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include "utils.h"

/**
 * @brief YOLOv8 constructor. Loads a TensorRT engine from a model path and config.
 * @param trtModelPath Path to TensorRT engine file.
 * @param config YOLOv8 configuration parameters.
 * @throws std::runtime_error if the engine cannot be loaded.
 */
YOLOv8::YOLOv8(const std::string &trtModelPath, const Config &config)
    : mDetectionThreshold(config.probabilityThreshold), mNMSThreshold(config.nmsThreshold), mTopK(config.topK),
      mSegChannels(config.segChannels), mSegHeight(config.segH), mSegWidth(config.segW), mSegThreshold(config.segmentationThreshold),
      mClassNames(config.classNames), mNumKPS(config.numKPS), mKPSThresh(config.kpsThreshold)
{
    if (!trtModelPath.empty())
    {
        // If no ONNX model, check for TRT model
        // Load the TensorRT engine file directly
        mEngine = std::make_unique<TRTEngine<float>>(trtModelPath);
        bool succ = mEngine->loadNetwork(trtModelPath, mMean, mStd, mNormalize);
        if (!succ)
        {
            throw std::runtime_error("Error: Unable to load TensorRT engine from " + trtModelPath);
        }
    }
    else
    {
        throw std::runtime_error("Error: Neither ONNX model nor TensorRT engine path provided.");
    }
}

/**
 * @brief Preprocesses a CUDA BGR image for YOLOv8 inference.
 * @param gpuImg Input CUDA image (BGR).
 * @return Nested vector suitable for batch inference input.
 */
std::vector<std::vector<cv::cuda::GpuMat>> YOLOv8::preprocess(const cv::cuda::GpuMat &gpuImg)
{
    // Populate the input vectors
    const auto &inputDims = mEngine->getInputDims();

    // Convert the image from BGR to RGB
    cv::cuda::GpuMat rgbMat;
    cv::cuda::cvtColor(gpuImg, rgbMat, cv::COLOR_BGR2RGB);

    auto resized = rgbMat;

    // Resize to the model expected input size while maintaining the aspect ratio with the use of padding
    if (resized.rows != inputDims[0].d[1] || resized.cols != inputDims[0].d[2])
    {
        // Only resize if not already the right size to avoid unecessary copy
        resized = resizeKeepAspectRatioPadRightBottom(rgbMat, inputDims[0].d[1], inputDims[0].d[2]);
    }

    // Convert to format expected by our inference engine
    // The reason for the strange format is because it supports models with multiple inputs as well as batching
    // In our case though, the model only has a single input and we are using a batch size of 1.
    std::vector<cv::cuda::GpuMat> input{std::move(resized)};
    std::vector<std::vector<cv::cuda::GpuMat>> inputs{std::move(input)};

    // These params will be used in the post-processing stage
    mInputImgHeight = rgbMat.rows;
    mInputImgWidth = rgbMat.cols;
    mAspectScaleFactor = 1.f / std::min(inputDims[0].d[2] / static_cast<float>(rgbMat.cols), inputDims[0].d[1] / static_cast<float>(rgbMat.rows));

    return inputs;
}

/**
 * @brief Run YOLOv8 inference and return detected objects (CUDA input).
 * @param inputImageBGR Input CUDA image (BGR).
 * @return Vector of detected objects.
 */
std::vector<Object> YOLOv8::detectObjects(const cv::cuda::GpuMat &inputImageBGR)
{
    // Preprocess the input image
#ifdef ENABLE_BENCHMARKS
    static int numIts = 1;
    preciseStopwatch s1;
#endif
    const auto input = preprocess(inputImageBGR);
#ifdef ENABLE_BENCHMARKS
    static long long t1 = 0;
    t1 += s1.elapsedTime<long long, std::chrono::microseconds>();
    std::cout << "Avg Preprocess time: " << (t1 / numIts) / 1000.f << " ms" << std::endl;
#endif
    // Run inference using the TensorRT engine
#ifdef ENABLE_BENCHMARKS
    preciseStopwatch s2;
#endif
    std::vector<std::vector<std::vector<float>>> featureVectors;
    auto succ = mEngine->runInference(input, featureVectors);
    if (!succ)
    {
        throw std::runtime_error("Error: Unable to run inference.");
    }

#ifdef ENABLE_BENCHMARKS
    static long long t2 = 0;
    t2 += s2.elapsedTime<long long, std::chrono::microseconds>();
    std::cout << "Avg Inference time: " << (t2 / numIts) / 1000.f << " ms" << std::endl;
    preciseStopwatch s3;
#endif
    // Check if our model does only object detection or also supports segmentation
    std::vector<Object> ret;
    const auto &numOutputs = mEngine->getOutputDims().size();
    if (numOutputs == 1)
    {
        // Object detection or pose estimation
        // Since we have a batch size of 1 and only 1 output, we must convert the output from a 3D array to a 1D array.
        std::vector<float> featureVector;
        transformOutput(featureVectors, featureVector);

        const auto &outputDims = mEngine->getOutputDims();
        int numChannels = outputDims[outputDims.size() - 1].d[1];
        // TODO: Need to improve this to make it more generic (don't use magic number).
        // For now it works with Ultralytics pretrained models.
        if (numChannels == 56)
        {
            // Pose estimation
            ret = postprocessPose(featureVector);
        }
        else
        {
            // Object detection
            ret = postprocessDetect(featureVector);
        }
    }
    else
    {
        // Segmentation
        // Since we have a batch size of 1 and 2 outputs, we must convert the output from a 3D array to a 2D array.
        std::vector<std::vector<float>> featureVector;
        transformOutput(featureVectors, featureVector);
        ret = postProcessSegmentation(featureVector);
    }
#ifdef ENABLE_BENCHMARKS
    static long long t3 = 0;
    t3 += s3.elapsedTime<long long, std::chrono::microseconds>();
    std::cout << "Avg Postprocess time: " << (t3 / numIts++) / 1000.f << " ms\n"
              << std::endl;
#endif
    return ret;
}

/**
 * @brief Run YOLOv8 inference and return detected objects (CPU input).
 * @param inputImageBGR Input OpenCV image (BGR, CPU memory).
 * @return Vector of detected objects.
 */
std::vector<Object> YOLOv8::detectObjects(const cv::Mat &inputImageBGR)
{
    // Upload the image to GPU memory
    cv::cuda::GpuMat gpuImg;
    gpuImg.upload(inputImageBGR);

    // Call detectObjects with the GPU image
    return detectObjects(gpuImg);
}

/**
 * @brief Post-process segmentation output tensors into object vector (YOLOv8 Segmentation).
 * @param featureVectors Model output tensors (segmentation).
 * @return Vector of detected objects with segmentation masks.
 */
std::vector<Object> YOLOv8::postProcessSegmentation(std::vector<std::vector<float>> &featureVectors)
{
    const auto &outputDims = mEngine->getOutputDims();

    int numChannels = outputDims[0].d[1];
    int numAnchors = outputDims[0].d[2];

    const auto numClasses = numChannels - -4;

    // Ensure the output lengths are correct
    if (featureVectors[0].size() != static_cast<size_t>(numChannels) * numAnchors)
    {
        throw std::logic_error("Output at index 0 has incorrect length");
    }

    if (featureVectors[1].size() != static_cast<size_t>(mSegChannels) * mSegHeight * mSegWidth)
    {
        throw std::logic_error("Output at index 1 has incorrect length");
    }

    cv::Mat output = cv::Mat(numChannels, numAnchors, CV_32F, featureVectors[0].data());
    output = output.t();

    cv::Mat protos = cv::Mat(mSegChannels, mSegHeight * mSegWidth, CV_32F, featureVectors[1].data());

    std::vector<int> labels;
    std::vector<float> scores;
    std::vector<cv::Rect> bboxes;
    std::vector<cv::Mat> maskConfs;
    std::vector<int> indices;

    // Object the bounding boxes and class labels
    for (int i = 0; i < numAnchors; i++)
    {
        auto rowPtr = output.row(i).ptr<float>();
        auto bboxesPtr = rowPtr;
        auto scoresPtr = rowPtr + 4;
        auto maskConfsPtr = rowPtr + 4 + numClasses;
        auto maxSPtr = std::max_element(scoresPtr, scoresPtr + numClasses);
        float score = *maxSPtr;
        if (score > mDetectionThreshold)
        {
            float x = *bboxesPtr++;
            float y = *bboxesPtr++;
            float w = *bboxesPtr++;
            float h = *bboxesPtr;

            float x0 = std::clamp((x - 0.5f * w) * mAspectScaleFactor, 0.f, mInputImgWidth);
            float y0 = std::clamp((y - 0.5f * h) * mAspectScaleFactor, 0.f, mInputImgHeight);
            float x1 = std::clamp((x + 0.5f * w) * mAspectScaleFactor, 0.f, mInputImgWidth);
            float y1 = std::clamp((y + 0.5f * h) * mAspectScaleFactor, 0.f, mInputImgHeight);

            int label = maxSPtr - scoresPtr;
            cv::Rect_<float> bbox;
            bbox.x = x0;
            bbox.y = y0;
            bbox.width = x1 - x0;
            bbox.height = y1 - y0;

            cv::Mat maskConf = cv::Mat(1, mSegChannels, CV_32F, maskConfsPtr);

            bboxes.push_back(bbox);
            labels.push_back(label);
            scores.push_back(score);
            maskConfs.push_back(maskConf);
        }
    }

    // Require OpenCV 4.7 for this function
    cv::dnn::NMSBoxesBatched(bboxes, scores, labels, mDetectionThreshold, mNMSThreshold, indices);

    // Obtain the segmentation masks
    cv::Mat masks;
    std::vector<Object> objs;
    int cnt = 0;
    for (auto &i : indices)
    {
        if (cnt >= mTopK)
        {
            break;
        }
        cv::Rect tmp = bboxes[i];
        Object obj;
        obj.label = labels[i];
        obj.rect = tmp;
        obj.probability = scores[i];
        masks.push_back(maskConfs[i]);
        objs.push_back(obj);
        cnt += 1;
    }

    // Convert segmentation mask to original frame
    if (!masks.empty())
    {
        cv::Mat matmulRes = (masks * protos).t();
        cv::Mat maskMat = matmulRes.reshape(indices.size(), {mSegWidth, mSegHeight});

        std::vector<cv::Mat> maskChannels;
        cv::split(maskMat, maskChannels);
        const auto inputDims = mEngine->getInputDims();

        cv::Rect roi;
        if (mInputImgHeight > mInputImgWidth)
        {
            roi = cv::Rect(0, 0, mSegWidth * mInputImgWidth / mInputImgHeight, mSegHeight);
        }
        else
        {
            roi = cv::Rect(0, 0, mSegWidth, mSegHeight * mInputImgHeight / mInputImgWidth);
        }

        for (size_t i = 0; i < indices.size(); i++)
        {
            cv::Mat dest, mask;
            cv::exp(-maskChannels[i], dest);
            dest = 1.0 / (1.0 + dest);
            dest = dest(roi);
            cv::resize(dest, mask, cv::Size(static_cast<int>(mInputImgWidth), static_cast<int>(mInputImgHeight)), cv::INTER_LINEAR);
            objs[i].boxMask = mask(objs[i].rect) > mSegThreshold;
        }
    }

    return objs;
}

/**
 * @brief Post-process pose estimation output tensors into object vector.
 * @param featureVector Model output tensor.
 * @return Vector of detected objects with keypoints.
 */
std::vector<Object> YOLOv8::postprocessPose(std::vector<float> &featureVector)
{
    const auto &outputDims = mEngine->getOutputDims();
    auto numChannels = outputDims[0].d[1];
    auto numAnchors = outputDims[0].d[2];

    std::vector<cv::Rect> bboxes;
    std::vector<float> scores;
    std::vector<int> labels;
    std::vector<int> indices;
    std::vector<std::vector<float>> kpss;

    cv::Mat output = cv::Mat(numChannels, numAnchors, CV_32F, featureVector.data());
    output = output.t();

    // Get all the YOLO proposals
    for (int i = 0; i < numAnchors; i++)
    {
        auto rowPtr = output.row(i).ptr<float>();
        auto bboxesPtr = rowPtr;
        auto scoresPtr = rowPtr + 4;
        auto kps_ptr = rowPtr + 5;
        float score = *scoresPtr;
        if (score > mDetectionThreshold)
        {
            float x = *bboxesPtr++;
            float y = *bboxesPtr++;
            float w = *bboxesPtr++;
            float h = *bboxesPtr;

            float x0 = std::clamp((x - 0.5f * w) * mAspectScaleFactor, 0.f, mInputImgWidth);
            float y0 = std::clamp((y - 0.5f * h) * mAspectScaleFactor, 0.f, mInputImgHeight);
            float x1 = std::clamp((x + 0.5f * w) * mAspectScaleFactor, 0.f, mInputImgWidth);
            float y1 = std::clamp((y + 0.5f * h) * mAspectScaleFactor, 0.f, mInputImgHeight);

            cv::Rect_<float> bbox;
            bbox.x = x0;
            bbox.y = y0;
            bbox.width = x1 - x0;
            bbox.height = y1 - y0;

            std::vector<float> kps;
            for (int k = 0; k < mNumKPS; k++)
            {
                float kpsX = *(kps_ptr + 3 * k) * mAspectScaleFactor;
                float kpsY = *(kps_ptr + 3 * k + 1) * mAspectScaleFactor;
                float kpsS = *(kps_ptr + 3 * k + 2);
                kpsX = std::clamp(kpsX, 0.f, mInputImgWidth);
                kpsY = std::clamp(kpsY, 0.f, mInputImgHeight);
                kps.push_back(kpsX);
                kps.push_back(kpsY);
                kps.push_back(kpsS);
            }

            bboxes.push_back(bbox);
            labels.push_back(0); // All detected objects are people
            scores.push_back(score);
            kpss.push_back(kps);
        }
    }

    // Run NMS
    cv::dnn::NMSBoxesBatched(bboxes, scores, labels, mDetectionThreshold, mNMSThreshold, indices);

    std::vector<Object> objects;

    // Choose the top k detections
    int cnt = 0;
    for (auto &chosenIdx : indices)
    {
        if (cnt >= mTopK)
        {
            break;
        }

        Object obj{};
        obj.probability = scores[chosenIdx];
        obj.label = labels[chosenIdx];
        obj.rect = bboxes[chosenIdx];
        obj.kps = kpss[chosenIdx];
        objects.push_back(obj);

        cnt += 1;
    }

    return objects;
}

/**
 * @brief Post-process detection output tensors into object vector.
 * @param featureVector Model output tensor.
 * @return Vector of detected objects with bounding boxes.
 */
std::vector<Object> YOLOv8::postprocessDetect(std::vector<float> &featureVector)
{
    const auto &outputDims = mEngine->getOutputDims();

    auto numChannels = outputDims[0].d[1];
    auto numAnchors = outputDims[0].d[2];
    auto numClasses = mClassNames.size();

    std::vector<cv::Rect> bboxes;
    std::vector<float> scores;
    std::vector<int> labels;
    std::vector<int> indices;

    cv::Mat output = cv::Mat(numChannels, numAnchors, CV_32F, featureVector.data());
    output = output.t();

    // Get all the YOLO proposals
    for (int i = 0; i < numAnchors; i++)
    {
        auto rowPtr = output.row(i).ptr<float>();
        auto bboxesPtr = rowPtr;
        auto scoresPtr = rowPtr + 4;
        auto maxSPtr = std::max_element(scoresPtr, scoresPtr + numClasses);
        float score = *maxSPtr;
        if (score > mDetectionThreshold)
        {
            float x = *bboxesPtr++;
            float y = *bboxesPtr++;
            float w = *bboxesPtr++;
            float h = *bboxesPtr;

            float x0 = std::clamp((x - 0.5f * w) * mAspectScaleFactor, 0.f, mInputImgWidth);
            float y0 = std::clamp((y - 0.5f * h) * mAspectScaleFactor, 0.f, mInputImgHeight);
            float x1 = std::clamp((x + 0.5f * w) * mAspectScaleFactor, 0.f, mInputImgWidth);
            float y1 = std::clamp((y + 0.5f * h) * mAspectScaleFactor, 0.f, mInputImgHeight);

            int label = maxSPtr - scoresPtr;
            cv::Rect_<float> bbox;
            bbox.x = x0;
            bbox.y = y0;
            bbox.width = x1 - x0;
            bbox.height = y1 - y0;

            bboxes.push_back(bbox);
            labels.push_back(label);
            scores.push_back(score);
        }
    }

    // Run NMS
    cv::dnn::NMSBoxesBatched(bboxes, scores, labels, mDetectionThreshold, mNMSThreshold, indices);

    std::vector<Object> objects;

    // Choose the top k detections
    int cnt = 0;
    for (auto &chosenIdx : indices)
    {
        if (cnt >= mTopK)
        {
            break;
        }

        Object obj{};
        obj.probability = scores[chosenIdx];
        obj.label = labels[chosenIdx];
        obj.rect = bboxes[chosenIdx];
        objects.push_back(obj);

        cnt += 1;
    }

    return objects;
}

/**
 * @brief Draw object bounding boxes, labels, and masks (if present) on an image.
 * @param image OpenCV image (CPU memory) to annotate.
 * @param objects Vector of detected objects.
 * @param scale Drawing scale (default: 1).
 */
void YOLOv8::drawObjectLabels(cv::Mat &image, const std::vector<Object> &objects, unsigned int scale)
{
    // If segmentation information is present, start with that
    if (!objects.empty() && !objects[0].boxMask.empty())
    {
        cv::Mat mask = image.clone();
        for (const auto &object : objects)
        {
            // Choose the color
            int colorIndex = object.label % mColorList.size(); // We have only defined 80 unique colors
            cv::Scalar color = cv::Scalar(mColorList[colorIndex][0], mColorList[colorIndex][1], mColorList[colorIndex][2]);

            // Add the mask for said object
            mask(object.rect).setTo(color * 255, object.boxMask);
        }
        // Add all the masks to our image
        cv::addWeighted(image, 0.5, mask, 0.8, 1, image);
    }

    // Bounding boxes and annotations
    for (auto &object : objects)
    {
        // Choose the color
        int colorIndex = object.label % mColorList.size(); // We have only defined 80 unique colors
        cv::Scalar color = cv::Scalar(mColorList[colorIndex][0], mColorList[colorIndex][1], mColorList[colorIndex][2]);
        float meanColor = cv::mean(color)[0];
        cv::Scalar txtColor;
        if (meanColor > 0.5)
        {
            txtColor = cv::Scalar(0, 0, 0);
        }
        else
        {
            txtColor = cv::Scalar(255, 255, 255);
        }

        const auto &rect = object.rect;

        // Draw rectangles and text
        char text[256];
        sprintf(text, "%s %.1f%%", mClassNames[object.label].c_str(), object.probability * 100);

        int baseLine = 0;
        cv::Size labelSize = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.35 * scale, scale, &baseLine);

        cv::Scalar txt_bk_color = color * 0.7 * 255;

        int x = object.rect.x;
        int y = object.rect.y + 1;

        cv::rectangle(image, rect, color * 255, scale + 1);

        cv::rectangle(image, cv::Rect(cv::Point(x, y), cv::Size(labelSize.width, labelSize.height + baseLine)), txt_bk_color, -1);

        cv::putText(image, text, cv::Point(x, y + labelSize.height), cv::FONT_HERSHEY_SIMPLEX, 0.35 * scale, txtColor, scale);

        // Pose estimation
        if (!object.kps.empty())
        {
            auto &kps = object.kps;
            for (int k = 0; k < mNumKPS + 2; k++)
            {
                if (k < mNumKPS)
                {
                    int kpsX = std::round(kps[k * 3]);
                    int kpsY = std::round(kps[k * 3 + 1]);
                    float kpsS = kps[k * 3 + 2];
                    if (kpsS > mKPSThresh)
                    {
                        cv::Scalar kpsColor = cv::Scalar(mKPSColors[k][0], mKPSColors[k][1], mKPSColors[k][2]);
                        cv::circle(image, {kpsX, kpsY}, 5, kpsColor, -1);
                    }
                }
                auto &ske = mSkeleton[k];
                int pos1X = std::round(kps[(ske[0] - 1) * 3]);
                int pos1Y = std::round(kps[(ske[0] - 1) * 3 + 1]);

                int pos2X = std::round(kps[(ske[1] - 1) * 3]);
                int pos2Y = std::round(kps[(ske[1] - 1) * 3 + 1]);

                float pos1S = kps[(ske[0] - 1) * 3 + 2];
                float pos2S = kps[(ske[1] - 1) * 3 + 2];

                if (pos1S > mKPSThresh && pos2S > mKPSThresh)
                {
                    cv::Scalar limbColor = cv::Scalar(mLimbColors[k][0], mLimbColors[k][1], mLimbColors[k][2]);
                    cv::line(image, {pos1X, pos1Y}, {pos2X, pos2Y}, limbColor, 2);
                }
            }
        }
    }
}