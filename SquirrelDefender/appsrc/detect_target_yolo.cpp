#ifdef ENABLE_CV
#if defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WIN) || defined(BLD_WSL)

/********************************************************************************
 * @file    detect_target_yolo.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   All methods needed to initialize and create a detection network and
            choose a target.
********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "detect_target_yolo.h"

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
std::vector<Object> g_yolo_detections;
int g_yolo_detection_count;

#elif defined(BLD_WIN)

cv::dnn::Net g_net;
std::vector<YoloNet::detection> g_yolo_detections;
int g_yolo_detection_count;

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
    if (g_valid_image_rcvd)
    {
#if defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WSL)

        g_yolo_detections = yolov8_detector->detectObjects(g_image);
        g_yolo_detection_count = g_yolo_detections.size();
        // yolov8_detector->drawObjectLabels(g_image, g_yolo_detections);

#elif defined(BLD_WIN)

        YoloNet::detect(g_image, g_net, g_yolo_detections);
        g_yolo_detection_count = g_yolo_detections.size();

#else

#error "Please define a build platform."

#endif
    }
}

/********************************************************************************
 * Function: YOLO
 * Description: Class constructor
 ********************************************************************************/
YOLO::YOLO(void) {};

/********************************************************************************
 * Function: ~YOLO
 * Description: Class destructor
 ********************************************************************************/
YOLO::~YOLO(void) {};

/********************************************************************************
 * Function: initialize_detection_net
 * Description: Delete detection network to free up resources.
 ********************************************************************************/
bool YOLO::init(void)
{
#if defined(BLD_JETSON_ORIN_NANO)

    YOLOv8::Config config;
    const std::string engine_path = "/workspace/OperationSquirrel/SquirrelDefender/models/yolov8s/yolov8s.engine.orin.fp16";
    yolov8_detector = new YOLOv8(engine_path, config);

    g_yolo_detections = std::vector<Object>();
    g_yolo_detections.reserve(100);

#elif defined(BLD_WSL)

    YOLOv8::Config config;
    const std::string engine_path = "/workspace/OperationSquirrel/SquirrelDefender/models/yolov8s/yolov8s.engine.NVIDIAGeForceRTX3060LaptopGPU.fp16";
    yolov8_detector = new YOLOv8(engine_path, config);

    g_yolo_detections = std::vector<Object>();
    g_yolo_detections.reserve(100);

#elif defined(BLD_WIN)

    const std::string class_list_path = "../../models/yolov5m/coco.names";
    const std::string model = "../../models/yolov5m/yolov5m.onnx";
    g_net = YoloNet::create(model, class_list_path, cv::dnn::DNN_BACKEND_CUDA, cv::dnn::DNN_TARGET_CUDA);
    g_yolo_detections = std::vector<YoloNet::detection>();
    g_yolo_detections.reserve(100);

#else

#error "Please define a build platform."

#endif

    return true;
}

/********************************************************************************
 * Function: detection_loop
 * Description: Process video stream and output detected objects.
 ********************************************************************************/
void YOLO::loop(void)
{
    detect_targets();
}

/********************************************************************************
 * Function: shutdown
 * Description: Shutdown detection network
 ********************************************************************************/
void YOLO::shutdown(void)
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

#endif // BLD_JETSON_ORIN_NANO
#endif // ENABLE_CV
