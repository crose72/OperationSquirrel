#include <opencv2/core/version.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <vpi/OpenCVInterop.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/opencv.hpp>

#include <vpi/Array.h>
#include <vpi/Image.h>
#include <vpi/Status.h>
#include <vpi/Stream.h>
#include <vpi/algo/KLTFeatureTracker.h>

#include <iostream>
#include <vector>
#include <map>

#define CHECK_STATUS(STMT) \
    do { \
        VPIStatus status = (STMT); \
        if (status != VPI_SUCCESS) { \
            char buffer[VPI_MAX_STATUS_MESSAGE_LENGTH]; \
            vpiGetLastStatusMessage(buffer, sizeof(buffer)); \
            throw std::runtime_error(vpiStatusGetName(status) + std::string(": ") + buffer); \
        } \
    } while (0)

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <cpu|pva|cuda>" << std::endl;
        return 1;
    }

    std::string strBackend = argv[1];
    VPIBackend backend;
    if (strBackend == "cpu") {
        backend = VPI_BACKEND_CPU;
    } else if (strBackend == "cuda") {
        backend = VPI_BACKEND_CUDA;
    } else if (strBackend == "pva") {
        backend = VPI_BACKEND_PVA;
    } else {
        std::cerr << "Invalid backend option. Use cpu, cuda, or pva." << std::endl;
        return 1;
    }

    // Define GStreamer pipeline for the CSI camera
    std::string gst_pipeline = 
        "nvarguscamerasrc ! video/x-raw(memory:NVMM), "
        "width=1280, height=720, framerate=30/1 ! nvvidconv ! "
        "video/x-raw, format=(string)BGRx ! videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink drop=true sync=false";

    // Open the CSI camera
    cv::VideoCapture cap(gst_pipeline, cv::CAP_GSTREAMER);
    
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera." << std::endl;
        return 1;
    }

    VPIStream stream = NULL;
    CHECK_STATUS(vpiStreamCreate(backend, &stream));

    cv::Mat frame;
    cap >> frame;
    if (frame.empty()) {
        std::cerr << "Error: Empty frame captured." << std::endl;
        return 1;
    }

    cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
    VPIImage imgTemplate = NULL, imgReference = NULL;
    CHECK_STATUS(vpiImageCreateWrapperOpenCVMat(frame, 0, &imgTemplate));
    CHECK_STATUS(vpiImageCreateWrapperOpenCVMat(frame, 0, &imgReference));

    std::vector<VPIKLTTrackedBoundingBox> bboxes;
    std::vector<VPIHomographyTransform2D> preds;

    VPIKLTTrackedBoundingBox testBox = {};
    testBox.bbox.xform.mat3[0][0] = 1;
    testBox.bbox.xform.mat3[1][1] = 1;
    testBox.bbox.xform.mat3[0][2] = 100;
    testBox.bbox.xform.mat3[1][2] = 100;
    testBox.bbox.width  = 50;
    testBox.bbox.height = 50;
    testBox.trackingStatus = 0;
    testBox.templateStatus = 1;
    bboxes.push_back(testBox);

    VPIHomographyTransform2D xform = {};
    xform.mat3[0][0] = 1;
    xform.mat3[1][1] = 1;
    xform.mat3[2][2] = 1;
    preds.push_back(xform);

    VPIArray inputBoxList = NULL, inputPredList = NULL, outputBoxList = NULL, outputEstimList = NULL;
    VPIPayload klt = NULL;
    VPIKLTFeatureTrackerParams params;
    CHECK_STATUS(vpiInitKLTFeatureTrackerParams(&params));

    CHECK_STATUS(vpiArrayCreate(128, VPI_ARRAY_TYPE_KLT_TRACKED_BOUNDING_BOX, 0, &outputBoxList));
    CHECK_STATUS(vpiArrayCreate(128, VPI_ARRAY_TYPE_HOMOGRAPHY_TRANSFORM_2D, 0, &outputEstimList));
    CHECK_STATUS(vpiCreateKLTFeatureTracker(backend, frame.cols, frame.rows, VPI_IMAGE_FORMAT_U8, NULL, &klt));

    VPIArrayData data = {};
    data.bufferType = VPI_ARRAY_BUFFER_HOST_AOS;
    data.buffer.aos.type = VPI_ARRAY_TYPE_KLT_TRACKED_BOUNDING_BOX;
    data.buffer.aos.capacity = bboxes.capacity();
    data.buffer.aos.sizePointer = new int32_t(bboxes.size());
    data.buffer.aos.data = &bboxes[0];
    CHECK_STATUS(vpiArrayCreateWrapper(&data, 0, &inputBoxList));

    data.buffer.aos.type = VPI_ARRAY_TYPE_HOMOGRAPHY_TRANSFORM_2D;
    data.buffer.aos.data = &preds[0];
    CHECK_STATUS(vpiArrayCreateWrapper(&data, 0, &inputPredList));

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
        CHECK_STATUS(vpiImageSetWrappedOpenCVMat(imgReference, frame));

        CHECK_STATUS(vpiSubmitKLTFeatureTracker(stream, backend, klt, imgTemplate, inputBoxList, inputPredList,
                                                imgReference, outputBoxList, outputEstimList, &params));
        CHECK_STATUS(vpiStreamSync(stream));

        VPIImageData imgData;
        CHECK_STATUS(vpiImageLockData(imgReference, VPI_LOCK_READ, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &imgData));

        cv::Mat debugFrame(imgData.buffer.pitch.planes[0].height,
                           imgData.buffer.pitch.planes[0].width,
                           CV_8U,
                           imgData.buffer.pitch.planes[0].data,
                           imgData.buffer.pitch.planes[0].pitchBytes);

        CHECK_STATUS(vpiImageUnlock(imgReference));

        for (const auto& box : bboxes) {
            if (box.trackingStatus == 0) {
                cv::rectangle(debugFrame, 
                              cv::Rect(box.bbox.xform.mat3[0][2],
                                       box.bbox.xform.mat3[1][2],
                                       box.bbox.width,
                                       box.bbox.height),
                              cv::Scalar(255, 255, 255), 2);
            }
        }

        cv::imshow("KLT Tracker Output", debugFrame);
        if (cv::waitKey(1) == 27) break;
    }

    vpiStreamDestroy(stream);
    vpiPayloadDestroy(klt);
    vpiArrayDestroy(inputBoxList);
    vpiArrayDestroy(inputPredList);
    vpiArrayDestroy(outputBoxList);
    vpiArrayDestroy(outputEstimList);
    vpiImageDestroy(imgReference);
    vpiImageDestroy(imgTemplate);

    return 0;
}
