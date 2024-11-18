#ifdef ENABLE_CV
#ifdef BLD_WIN

/********************************************************************************
 * @file    detect_target_yolo.cpp
 * @author  Cameron Rose
 * @date    6/7/2023
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
cv::dnn::Net net;
std::vector<yolo_net::detection> yolo_detections;
int yolo_detection_count;

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
    const std::string class_list_path = "../../networks/yolov5m/coco.names";
    const std::string model = "../../networks/yolov5m/yolov5m.onnx";
    net = yolo_net::create(model, class_list_path, cv::dnn::DNN_BACKEND_CUDA, cv::dnn::DNN_TARGET_CUDA);

    return true;
}

/********************************************************************************
 * Function: detection_loop
 * Description: Process video stream and output detected objects.
 ********************************************************************************/
void YOLO::loop(void)
{
    yolo_net::detect(image, net, yolo_detections);
    yolo_detection_count = yolo_detections.size();
}

/********************************************************************************
 * Function: shutdown
 * Description: Shutdown detection network
 ********************************************************************************/
void YOLO::shutdown(void)
{

}

#endif // BLD_WIN
#endif // ENABLE_CV
