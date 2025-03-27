#ifdef ENABLE_CV
#if defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WIN)

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
#ifdef BLD_JETSON_ORIN_NANO

YoloV8* yolov8_detector;
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
    #ifdef BLD_JETSON_ORIN_NANO

    YoloV8Config config;
    const std::string engine_path = "../networks/yolov8s/yolov8s.engine.Orin.fp16.1.1.-1.-1.-1";
    const std::string model_path = "../networks/yolov8s/yolov8s.onnx";
    yolov8_detector = new YoloV8(model_path, engine_path, config);
    
    g_yolo_detections = std::vector<Object>();
    g_yolo_detections.reserve(100);

    #elif defined(BLD_WIN)

    const std::string class_list_path = "../../networks/yolov5m/coco.names";
    const std::string model = "../../networks/yolov5m/yolov5m.onnx";
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
    if (g_valid_image_rcvd)
    {
    #ifdef BLD_JETSON_ORIN_NANO

        g_yolo_detections = yolov8_detector->detectObjects(g_image);
        yolov8_detector->drawObjectLabels(g_image, g_yolo_detections);
        g_yolo_detection_count = g_yolo_detections.size();

    #elif defined(BLD_WIN)

        YoloNet::detect(g_image, g_net, g_yolo_detections);
        g_yolo_detection_count = g_yolo_detections.size();

    #else

    #error "Please define a build platform."

    #endif
    }
}

/********************************************************************************
 * Function: shutdown
 * Description: Shutdown detection network
 ********************************************************************************/
void YOLO::shutdown(void)
{
    #ifdef BLD_JETSON_ORIN_NANO

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
