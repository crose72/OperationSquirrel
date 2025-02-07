#ifdef ENABLE_CV
#ifdef BLD_JETSON_ORIN_NANO

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
YoloV8* yolov8_detector;
cv::dnn::Net g_net;
std::vector<YoloNet::detection> g_yolo_detections;
int g_yolo_detection_count;

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
    const std::string engine = "../../networks/yolov8s/yolov8s.engine.Orin.fp16.1.1.-1.-1.-1";
    const std::string model = "../../networks/yolov5m/yolov8s.onnx";
    yolov8_detector = new YoloV8(model, trtModelPath, config);
    g_yolo_detections = std::vector<YoloNet::detection>();
    g_yolo_detections.reserve(100);

    return true;
}

/********************************************************************************
 * Function: detection_loop
 * Description: Process video stream and output detected objects.
 ********************************************************************************/
void YOLO::loop(void)
{
    const auto detections = yolov8_detector.detectObject(g_image);
    g_yolo_detection_count = g_yolo_detections.size();
}

/********************************************************************************
 * Function: shutdown
 * Description: Shutdown detection network
 ********************************************************************************/
void YOLO::shutdown(void)
{

}

#endif // BLD_JETSON_ORIN_NANO
#endif // ENABLE_CV
