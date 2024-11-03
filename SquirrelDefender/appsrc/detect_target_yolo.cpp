#ifdef ENABLE_CV
#ifdef _WIN32

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
 * Function: initialize_detection_net
 * Description: Delete detection network to free up resources.
 ********************************************************************************/
bool YOLO::init(void)
{
    /*if (!create_detection_network())
    {
        PrintPass::c_fprintf("Failed to create detection network");
        return false;
    }*/
    const std::string class_list_path = "../../networks/yolov5m/coco.names";
    const std::string model = "../../networks/yolov5m/yolov5m.onnx";
    net = yolo_net::create(model, class_list_path, cv::dnn::DNN_BACKEND_CUDA, cv::dnn::DNN_TARGET_CUDA);
    yolo_detection_count = 0;

    return true;
}

/********************************************************************************
 * Function: detection_loop
 * Description: Process video stream and output detected objects.
 ********************************************************************************/
void YOLO::loop(void)
{
    yolo_net::detect(image, yolo_detections);
    yolo_detection_count = yolo_detections.size();
}

/********************************************************************************
 * Function: shutdown
 * Description: Shutdown detection network
 ********************************************************************************/
void YOLO::shutdown(void)
{

}

#endif // _WIN32
#endif // ENABLE_CV
