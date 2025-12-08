#ifdef ENABLE_CV
#if defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WIN) || defined(BLD_WSL)

/********************************************************************************
 * @file    detector_yolo.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   This module initializes and manages the YOLO-based object detector.
 *          It wraps either the TensorRT YOLOv8 engine (Jetson/WSL) or the
 *          OpenCV DNN backend (Windows) and performs object detection on
 *          incoming camera frames. Designed for use in the embedded-style
 *          init/loop/shutdown pipeline.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include <opencv2/cudaimgproc.hpp>

#include "common_inc.h"
#include "detector_yolo.h"
#include "video_io.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
#if defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WSL)

YOLOv8 *yolov8_detector;
std::vector<Object> g_det_yolo_list;
int g_det_count;

#elif defined(BLD_WIN)

cv::dnn::Net g_det_yolo_net;
std::vector<YoloNet::detection> g_det_yolo_list;
int g_det_count;

#else

#error "Please define a build platform."

#endif

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/
void detect_targets(void);

/********************************************************************************
 * Function: detect_targets
 * Description: Run object detection on image and return the detections and
 *              container of detected objects.
 ********************************************************************************/
void detect_targets(void)
{
    if (g_cam0_img_valid)
    {
#if defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WSL)

        g_det_yolo_list = yolov8_detector->detectObjects(g_cam0_img_cpu);
        g_det_count = g_det_yolo_list.size();

        if (g_app_use_video_playback)
        {
            yolov8_detector->drawObjectLabels(g_cam0_img_cpu, g_det_yolo_list);
        }

#elif defined(BLD_WIN)

        YoloNet::detect(g_cam0_img_cpu, g_det_yolo_net, g_det_yolo_list);
        g_det_count = g_det_yolo_list.size();

#else

#error "Please define a build platform."

#endif
    }
}

/********************************************************************************
 * Function: DetectorYOLO
 * Description: Class constructor
 ********************************************************************************/
DetectorYOLO::DetectorYOLO() {};

/********************************************************************************
 * Function: ~DetectorYOLO
 * Description: Class destructor
 ********************************************************************************/
DetectorYOLO::~DetectorYOLO() {};

/********************************************************************************
 * Function: initialize_detection_net
 * Description: Delete detection network to free up resources.
 ********************************************************************************/
bool DetectorYOLO::init(void)
{
#if defined(BLD_JETSON_ORIN_NANO)

    YOLOv8::Config config;
    const std::string engine_path = "/workspace/operationsquirrel/squirreldefender/networks/yolov8s/yolov8s.engine.orin.fp16";
    yolov8_detector = new YOLOv8(engine_path, config);

    g_det_yolo_list = std::vector<Object>();
    g_det_yolo_list.reserve(100);

#elif defined(BLD_WSL)

    YOLOv8::Config config;
    const std::string engine_path = "/workspace/operationsquirrel/squirreldefender/networks/yolov8s/yolov8s.engine.NVIDIAGeForceRTX3060LaptopGPU.fp16.batch1";
    yolov8_detector = new YOLOv8(engine_path, config);

    g_det_yolo_list = std::vector<Object>();
    g_det_yolo_list.reserve(100);

#elif defined(BLD_WIN)

    const std::string class_list_path = "../../networks/yolov5m/coco.names";
    const std::string model = "../../networks/yolov5m/yolov5m.onnx";
    g_det_yolo_net = YoloNet::create(model, class_list_path, cv::dnn::DNN_BACKEND_CUDA, cv::dnn::DNN_TARGET_CUDA);
    g_det_yolo_list = std::vector<YoloNet::detection>();
    g_det_yolo_list.reserve(100);

#else

#error "Please define a build platform."

#endif

    return true;
}

/********************************************************************************
 * Function: detection_loop
 * Description: Process video stream and output detected objects.
 ********************************************************************************/
void DetectorYOLO::loop(void)
{
    detect_targets();
}

/********************************************************************************
 * Function: shutdown
 * Description: Shutdown detection network
 ********************************************************************************/
void DetectorYOLO::shutdown(void)
{
#if defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WSL)

    delete yolov8_detector;
    yolov8_detector = nullptr;

#elif defined(BLD_WIN)

#warning "Remember to de-allocate memory at the end of the program if needed."

#else

#error "Please define a build platform."

#endif
}

#endif // defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WIN) || defined(BLD_WSL)
#endif // ENABLE_CV
