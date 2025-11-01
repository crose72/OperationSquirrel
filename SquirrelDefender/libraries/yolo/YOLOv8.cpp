#include "YOLOv8.h"
#include <algorithm>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include "utils.h"

/**
 * @brief YOLOv8 constructor. Loads a TensorRT engine and parameters
 *        from an engine path and config.
 * @param trtModelPath Path to TensorRT engine file.
 * @param config YOLOv8 configuration parameters.
 */
YOLOv8::YOLOv8(const std::string &trtModelPath, const Config &config)
    : mDetectionThreshold(config.probabilityThreshold),
      mNMSThreshold(config.nmsThreshold),
      mTopK(config.topK),
      mSegChannels(config.segChannels),
      mSegHeight(config.segH),
      mSegWidth(config.segW),
      mSegThreshold(config.segmentationThreshold),
      mClassNames(config.classNames),
      mNumKPS(config.numKPS),
      mKPSThresh(config.kpsThreshold)
{
    if (!trtModelPath.empty())
    {
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

    // Get engine parameters
    const std::vector<nvinfer1::Dims> inputDims = mEngine->getInputDims();

    if (inputDims[0].nbDims == 4)
    {
        // [N, C, H, W]
        mEngineInputHeight = inputDims[0].d[2];
        mEngineInputWidth = inputDims[0].d[3];
    }
    else if (inputDims[0].nbDims == 3)
    {
        // [C, H, W] (single image, N omitted)
        mEngineInputHeight = inputDims[0].d[1];
        mEngineInputWidth = inputDims[0].d[2];
    }
    else
    {
        // Unexpected engine format
        const std::string msg = "Unsupported input tensor for YOLO, actual input tensors is: " + std::to_string(inputDims[0].nbDims);
        spdlog::error(msg);
    }

    const std::vector<nvinfer1::Dims> outputDims = mEngine->getOutputDims();

    mNumOutputTensors = outputDims.size();

    for (int tensor = 0; tensor < mNumOutputTensors; ++tensor)
    {
        if (outputDims[tensor].nbDims == 3)
        {
            // Proposal output: [batch, anchor_features, num_anchors]
            mEngineBatchSize = outputDims[tensor].d[0];
            mNumAnchorFeatures = outputDims[tensor].d[1];
            mNumAnchors = outputDims[tensor].d[2];
        }
        else if (outputDims[tensor].nbDims == 4)
        {
            // Mask protos: [batch, seg_channels, seg_height, seg_width]
            // mSegBatchSize = outputDims[tensor].d[0];
            mSegChannels = outputDims[tensor].d[1];
            mSegHeight = outputDims[tensor].d[2];
            mSegWidth = outputDims[tensor].d[3];
        }
        else
        {
            // Unexpected engine format
            const std::string msg = "Unsupported output tensor for YOLO, actual output tensors is: " + std::to_string(outputDims[tensor].nbDims);
            spdlog::error(msg);
        }
    }

    mNumClasses = mClassNames.size();
}

/**
 * @brief Preprocesses a CUDA BGR image for YOLOv8 inference as a batch of 1.
 * @param gpuImg Input CUDA image (BGR).
 * @return Nested vector of resized single image with RGB format and padding
 */
std::vector<std::vector<cv::cuda::GpuMat>> YOLOv8::preprocess(const cv::cuda::GpuMat &gpuImg)
{
    std::vector<cv::cuda::GpuMat> inputImg{gpuImg};

    std::vector<std::vector<cv::cuda::GpuMat>> preprocessedBatch = preprocess(inputImg);

    return preprocessedBatch;
}

/**
 * @brief Preprocesses a batch of CUDA BGR images for YOLOv8 inference.
 * @param gpuImg Input vector of CUDA images (BGR).
 * @return Nested vector of resized images with RGB format and padding
 */
std::vector<std::vector<cv::cuda::GpuMat>> YOLOv8::preprocess(const std::vector<cv::cuda::GpuMat> &gpuImgs)
{
    mActualBatchSize = gpuImgs.size();
    mInputImgHeights.clear();
    mInputImgWidths.clear();
    mAspectScaleFactors.clear();

    std::vector<cv::cuda::GpuMat> resizedImgs;

    for (const auto &gpuImg : gpuImgs)
    {
        float inputImgHeight = gpuImg.rows;
        float inputImgWidth = gpuImg.cols;

        // Convert the image from BGR to RGB
        // This assumes that the image is being read as the OpenCV standard BGR format
        cv::cuda::GpuMat rgbMat;
        cv::cuda::cvtColor(gpuImg, rgbMat, cv::COLOR_BGR2RGB);

        // Initialize the resized image
        cv::cuda::GpuMat resizedImg = rgbMat;

        // Resize to the model expected input size while maintaining the
        // aspect ratio with the use of padding
        if (inputImgHeight != mEngineInputHeight || inputImgWidth != mEngineInputWidth)
        {
            resizedImg = letterbox(rgbMat, mEngineInputHeight, mEngineInputWidth);
        }

        resizedImgs.push_back(resizedImg);

        // Save the size of the image
        // These params will be used in the post-processing stage
        mInputImgHeights.push_back(inputImgHeight);
        mInputImgWidths.push_back(inputImgWidth);

        // How much the image is scaled up/down to required YOLO size
        mAspectScaleFactors.push_back(
            (float)1.0 / std::min(mEngineInputWidth / static_cast<float>(inputImgWidth),
                                  mEngineInputHeight / static_cast<float>(inputImgHeight)));
    }

    // For YOLOv8, each element of the outer vector is a model input tensor (only 1 for normal YOLO).
    // Each inner vector is a batch. So we want [ [img1,img2,...,imgN] ]
    std::vector<std::vector<cv::cuda::GpuMat>> preprocessedBatch;
    preprocessedBatch.push_back(std::move(resizedImgs));

    return preprocessedBatch;
}

/**
 * @brief Run YOLOv8 inference and return detected objects for a single image (CPU input).
 * @param inputImgBGR Input OpenCV image (BGR, CPU memory).
 * @return Vector of detections for a single image.
 */
std::vector<Object> YOLOv8::detectObjects(const cv::Mat &inputImgBGR)
{
    // Upload the image to GPU memory
    std::vector<cv::cuda::GpuMat> gpuImgs;
    cv::cuda::GpuMat gpuImg;

    gpuImg.upload(inputImgBGR);
    gpuImgs.push_back(gpuImg);

    std::vector<std::vector<Object>> detections = detectObjects(gpuImgs);
    return detections[0];
}

/**
 * @brief Run YOLOv8 inference and return detected objects for a single image (GPU input).
 * @param batchInputImgsBGR Input OpenCV images (BGR, CPU memory).
 * @return Vector of detections for a single image.
 */
std::vector<Object> YOLOv8::detectObjects(const cv::cuda::GpuMat &inputImageBGR)
{
    std::vector<cv::cuda::GpuMat> gpuImgs;

    gpuImgs.push_back(inputImageBGR);

    std::vector<std::vector<Object>> detections = detectObjects(gpuImgs);

    return detections[0];
}

/**
 * @brief Run YOLOv8 inference and return detected objects for a batch of images (CPU input).
 * @param batchInputImgsBGR Input OpenCV images (BGR, CPU memory).
 * @return Vector of detections for a batch of images.
 */
std::vector<std::vector<Object>> YOLOv8::detectObjects(const std::vector<cv::Mat> &batchInputImgsBGR)
{
    // Upload the image to GPU memory
    std::vector<cv::cuda::GpuMat> gpuImgs;

    for (const auto &cpuImg : batchInputImgsBGR)
    {
        cv::cuda::GpuMat gpuImg;
        gpuImg.upload(cpuImg);

        gpuImgs.push_back(gpuImg);
    }

    // Call detectObjects with the batch of GPU images
    return detectObjects(gpuImgs);
}

/**
 * @brief Run YOLOv8 inference and return detected objects for a batch of images (CPU input).
 * @param batchInputImgsBGR Input OpenCV images (BGR, CPU memory).
 * @return Vector of detections for a batch of images.
 */
std::vector<std::vector<Object>> YOLOv8::detectObjects(const std::vector<cv::cuda::GpuMat> &batchInputImgsBGR)
{
    // Preprocess the input image
#ifdef ENABLE_BENCHMARKS
    static int numIts = 1;
    preciseStopwatch s1;
#endif

    const std::vector<std::vector<cv::cuda::GpuMat>> engineInputBatch = preprocess(batchInputImgsBGR);

    // End timer to clock preprocessing time
#ifdef ENABLE_BENCHMARKS
    static long long t1 = 0;
    t1 += s1.elapsedTime<long long, std::chrono::microseconds>();
    spdlog::info("Avg Preprocess time: {:.3f} ms", (t1 / numIts) / 1000.f);
#endif

    // Start timer to clock inference time
#ifdef ENABLE_BENCHMARKS
    preciseStopwatch s2;
#endif

    // Run inference using the TensorRT engine
    // Raw engine outputs
    std::vector<std::vector<std::vector<float>>> featureVectors;

    bool inferenceSuccessful = mEngine->runInference(engineInputBatch, featureVectors);

    if (!inferenceSuccessful)
    {
        const std::string msg = "Error: Unable to run inference.";
        spdlog::error(msg);
    }

    // End timer to clock inference time
#ifdef ENABLE_BENCHMARKS
    static long long t2 = 0;
    t2 += s2.elapsedTime<long long, std::chrono::microseconds>();
    spdlog::info("Avg Inference time: {:.3f} ms", (t2 / numIts) / 1000.f);

    // Start timer to clock postprocessing time
    preciseStopwatch s3;
#endif

    // Nested vector for detections to support batch >= 1
    std::vector<std::vector<Object>> detections;

    // Check if our model does only object detection or also supports segmentation
    if (mNumOutputTensors == 1)
    {
        // Object detection or pose estimation
        // Since we have only 1 output from the model,
        // we must convert the output from a 3D array to a 1D array.
        std::vector<std::vector<float>> featureVector;
        transformOutput(featureVectors, featureVector);

        if (mNumAnchorFeatures == mNumPoseAnchorFeatures)
        {
            // Key pose estimation
            for (int i = 0; i < mActualBatchSize; ++i)
            {
                detections.push_back(postprocessPose(featureVector[i], i));
            }
        }
        else
        {
            // Object detection
            for (int i = 0; i < mActualBatchSize; ++i)
            {
                detections.push_back(postprocessDetect(featureVector[i], i));
            }
        }
    }
    else
    {
        // Segmentation
        for (int i = 0; i < mActualBatchSize; ++i)
        {
            detections.push_back(postProcessSegmentation(featureVectors[i], i));
        }
    }

    // End timer to clock postprocessing time
#ifdef ENABLE_BENCHMARKS
    static long long t3 = 0;
    t3 += s3.elapsedTime<long long, std::chrono::microseconds>();
    spdlog::info("Avg Postprocess time: {:.3f} ms", (t3 / numIts++) / 1000.f);
#endif

    return detections;
}

/**
 * @brief Post-process segmentation output tensors into object vector (YOLOv8 Segmentation).
 * @param featureVectors Model output tensors (segmentation).
 * @return Vector of detected objects with segmentation masks.
 */
std::vector<Object> YOLOv8::postProcessSegmentation(std::vector<std::vector<float>> &featureVectors, int imageInBatch)
{
    // Ensure the output lengths are correct
    if (featureVectors[0].size() != static_cast<size_t>(mNumAnchorFeatures) * mNumAnchors)
    {
        const std::string msg = "Output at index 0 has incorrect length";
        spdlog::error(msg);
    }

    if (featureVectors[1].size() != static_cast<size_t>(mSegChannels) * mSegHeight * mSegWidth)
    {

        const std::string msg = "Output at index 1 has incorrect length";
        spdlog::error(msg);
    }

    std::vector<int> labels;
    std::vector<float> scores;
    std::vector<cv::Rect> bboxes;
    std::vector<cv::Mat> maskConfs;
    std::vector<int> indices;

    // Convert output tensor 1 to opencv mat to obtain segmentation mask scores
    cv::Mat output = cv::Mat(mNumAnchorFeatures, mNumAnchors, CV_32F, featureVectors[0].data());
    output = output.t();

    decodeYOLOAnchors(
        featureVectors[0], &output, mNumAnchors, mNumClasses, mDetectionThreshold,
        mAspectScaleFactors[imageInBatch], mInputImgWidths[imageInBatch], mInputImgHeights[imageInBatch],
        bboxes, scores, labels, YOLO_SEG, &maskConfs, nullptr, mSegChannels, 0);

    // Run NMS - OpenCV 4.7 or later required
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
        // The second output tensor is used to apply the masks to each pixel identified as part of the
        // object in the image
        const cv::Mat protos = cv::Mat(mSegChannels, mSegHeight * mSegWidth, CV_32F, featureVectors[1].data());

        cv::Mat matmulRes = (masks * protos).t();
        cv::Mat maskMat = matmulRes.reshape(indices.size(), {mSegWidth, mSegHeight});

        std::vector<cv::Mat> maskChannels;
        cv::split(maskMat, maskChannels);

        cv::Rect roi;
        if (mInputImgHeights[imageInBatch] > mInputImgWidths[imageInBatch])
        {
            roi = cv::Rect(0, 0, mSegWidth * mInputImgWidths[imageInBatch] / mInputImgHeights[imageInBatch], mSegHeight);
        }
        else
        {
            roi = cv::Rect(0, 0, mSegWidth, mSegHeight * mInputImgHeights[imageInBatch] / mInputImgWidths[imageInBatch]);
        }

        for (size_t i = 0; i < indices.size(); i++)
        {
            cv::Mat dest, mask;
            cv::exp(-maskChannels[i], dest);
            dest = 1.0 / (1.0 + dest);
            dest = dest(roi);
            cv::resize(dest, mask, cv::Size(static_cast<int>(mInputImgWidths[imageInBatch]), static_cast<int>(mInputImgHeights[imageInBatch])), cv::INTER_LINEAR);
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
std::vector<Object> YOLOv8::postprocessPose(std::vector<float> &featureVector, int imageInBatch)
{
    std::vector<cv::Rect> bboxes;
    std::vector<float> scores;
    std::vector<int> labels;
    std::vector<int> indices;
    std::vector<std::vector<float>> kpss;

    decodeYOLOAnchors(
        featureVector, nullptr, mNumAnchors, mNumClasses, mDetectionThreshold,
        mAspectScaleFactors[imageInBatch], mInputImgWidths[imageInBatch], mInputImgHeights[imageInBatch],
        bboxes, scores, labels, YOLO_POSE, nullptr, &kpss, 0, mNumKPS);

    // Run NMS - OpenCV 4.7 or later required
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
std::vector<Object> YOLOv8::postprocessDetect(std::vector<float> &featureVector, int imageInBatch)
{
    std::vector<cv::Rect> bboxes;
    std::vector<float> scores;
    std::vector<int> labels;
    std::vector<int> indices;

    decodeYOLOAnchors(
        featureVector, nullptr, mNumAnchors, mNumClasses, mDetectionThreshold,
        mAspectScaleFactors[imageInBatch], mInputImgWidths[imageInBatch], mInputImgHeights[imageInBatch],
        bboxes, scores, labels, YOLO_DET, nullptr, nullptr, 0, 0);

    // Run NMS - OpenCV 4.7 or later required
    cv::dnn::NMSBoxesBatched(bboxes, scores, labels, mDetectionThreshold, mNMSThreshold, indices);
    std::vector<Object> objects;
    int cnt = 0;
    for (auto &chosenIdx : indices)
    {
        if (cnt >= mTopK)
            break;
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
 * @brief Decode YOLO output tensor anchors into detection, segmentation, or pose results.
 *
 * This function post-processes the output tensor from a YOLO model to extract object bounding boxes,
 * detection scores, class labels, and (optionally) segmentation masks or pose keypoints, according to the model type.
 *
 * @param output             Flattened output tensor (anchor-major, CHW format) from the model.
 * @param numAnchors         Number of anchor positions (e.g., 8400 for YOLOv8).
 * @param numClasses         Number of classes in the model (e.g., 80 for COCO).
 * @param detectionThreshold Minimum score to accept a detection.
 * @param aspectScaleFactor  Scale to convert normalized coordinates to image coordinates.
 * @param imgWidth           Width of the original input image (after aspect scaling).
 * @param imgHeight          Height of the original input image (after aspect scaling).
 * @param[out] bboxes        Output vector of detected bounding boxes (image coordinates).
 * @param[out] scores        Output vector of detection or instance scores.
 * @param[out] labels        Output vector of integer class labels.
 * @param type               Type of YOLO postprocess (YOLO_DET for detection, YOLO_SEG for segmentation, YOLO_POSE for pose).
 * @param[in,out] maskConfs  Optional pointer to output vector for segmentation mask coefficients (YOLO_SEG only). Pass nullptr if unused.
 * @param[in,out] kpss       Optional pointer to output vector for pose keypoints (YOLO_POSE only). Pass nullptr if unused.
 * @param numMaskChannels    Number of mask channels (YOLO_SEG only; set to 0 otherwise).
 * @param numKeypoints       Number of keypoints (YOLO_POSE only; set to 0 otherwise).
 *
 * @details
 *  - For object detection, fills `bboxes`, `scores`, and `labels`.
 *  - For segmentation, additionally fills `maskConfs` with per-object mask coefficients.
 *  - For pose estimation, additionally fills `kpss` with per-object keypoints.
 *
 *  The output tensor should be in anchor-major, CHW layout. Segmentation and pose features are only parsed if
 *  their respective pointers are non-null and type is set.
 */

void YOLOv8::decodeYOLOAnchors(
    const std::vector<float> &output,
    const cv::Mat *outputMat,
    int numAnchors,
    int numClasses,
    float detectionThreshold,
    float aspectScaleFactor,
    float imgWidth,
    float imgHeight,
    std::vector<cv::Rect> &bboxes,
    std::vector<float> &scores,
    std::vector<int> &labels,
    InferenceType type,
    // Optional: mask coeffs and keypoints containers (pass nullptr if unused)
    std::vector<cv::Mat> *maskConfs,
    std::vector<std::vector<float>> *kpss,
    int numMaskChannels,
    int numKeypoints)
{
    for (int anchor = 0; anchor < numAnchors; ++anchor)
    {
        // Fetch bbox from anchor-major layout
        float x = output[anchor + 0 * numAnchors];
        float y = output[anchor + 1 * numAnchors];
        float w = output[anchor + 2 * numAnchors];
        float h = output[anchor + 3 * numAnchors];

        // Fetch class scores and get label/score
        int class_start = 4 * numAnchors + anchor; // offset of class score for each anchor is after bbox dims
        float max_score = -1.f;
        int max_label = -1;

        // Detection/Segmentation: multi-class, find max
        if (type == YOLO_DET || type == YOLO_SEG)
        {
            for (int cls = 0; cls < numClasses; ++cls)
            {
                // Get class score indexed in the first output tensor
                // for
                float s = output[class_start + cls * numAnchors];
                if (s > max_score)
                {
                    max_score = s;
                    max_label = cls;
                }
            }
        }
        // Pose: typically one score (objectness)
        else if (type == YOLO_POSE)
        {
            max_score = output[class_start];
            max_label = 0; // only "person" class
        }

        if (max_score > detectionThreshold)
        {
            // Convert to absolute image coordinates
            float x0 = std::clamp((x - 0.5f * w) * aspectScaleFactor, 0.f, imgWidth);
            float y0 = std::clamp((y - 0.5f * h) * aspectScaleFactor, 0.f, imgHeight);
            float x1 = std::clamp((x + 0.5f * w) * aspectScaleFactor, 0.f, imgWidth);
            float y1 = std::clamp((y + 0.5f * h) * aspectScaleFactor, 0.f, imgHeight);

            bboxes.emplace_back(x0, y0, x1 - x0, y1 - y0);
            scores.push_back(max_score);
            labels.push_back(max_label);

            // --- Segmentation mask coefficients ---
            if (outputMat != nullptr && numMaskChannels > 0)
            {
                auto rowPtr = outputMat->row(anchor).ptr<float>();
                auto maskConfsPtr = rowPtr + 4 + numClasses;
                cv::Mat maskConf = cv::Mat(1, numMaskChannels, CV_32F, maskConfsPtr);
                maskConfs->push_back(maskConf); // .clone() is IMPORTANT!
            }

            // --- Keypoints for pose ---
            if (type == YOLO_POSE && kpss && numKeypoints > 0)
            {
                int kps_start = (4 + 1) * numAnchors + anchor; // 4 bbox + 1 obj score
                std::vector<float> kps;
                for (int k = 0; k < numKeypoints; ++k)
                {
                    float kpsX = output[kps_start + (3 * k + 0) * numAnchors] * aspectScaleFactor;
                    float kpsY = output[kps_start + (3 * k + 1) * numAnchors] * aspectScaleFactor;
                    float kpsS = output[kps_start + (3 * k + 2) * numAnchors];
                    kpsX = std::clamp(kpsX, 0.f, imgWidth);
                    kpsY = std::clamp(kpsY, 0.f, imgHeight);
                    kps.push_back(kpsX);
                    kps.push_back(kpsY);
                    kps.push_back(kpsS);
                }
                kpss->push_back(kps);
            }
        }
    }
}

/**
 * @brief Draw object bounding boxes, labels, and masks (if present) on an image.
 * @param image OpenCV image (CPU memory) to annotate.
 * @param objects Vector of detected objects.
 * @param scale Drawing scale (line thickness) (default: 1).
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
