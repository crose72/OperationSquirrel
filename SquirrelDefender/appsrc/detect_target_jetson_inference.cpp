#ifdef ENABLE_CV
#ifdef BLD_JETSON_B01

/********************************************************************************
 * @file    detect_target_jetson_inference.cpp
 * @author  Cameron Rose
 * @date    6/7/2023
 * @brief   This file contains the functions to initialize, run, and clean up
            object detection code.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "detect_target_jetson_inference.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
detectNet *net;
detectNet::Detection *detections;
int detection_count;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/
bool create_detection_network(void);
void detect_objects(void);

/********************************************************************************
 * Function: create_detection_network
 * Description: Initialize the network used for object detection.
 ********************************************************************************/
bool create_detection_network(void)
{
#ifdef DEBUG_BUILD

    detect_target_jetson_inference.h detection_params("../params.json");

    float detection_thresh = detection_params.get_float_param("Detection_tracking", "Detect_Thresh");
    uint32_t max_batch_size = detection_params.get_uint32_param("Detection_tracking", "Max_Batch_Size");
    uint32_t min_frames = detection_params.get_uint32_param("Detection_tracking", "Min_Frames");
    uint32_t drop_frames = detection_params.get_uint32_param("Detection_tracking", "Drop_Frames");
    float overlap_thresh = detection_params.get_float_param("Detection_tracking", "Overlap_Threshold");

#else

    float detection_thresh = (float)0.5;
    uint32_t max_batch_size = (uint32_t)4;
    uint32_t min_frames = (uint32_t)25;
    uint32_t drop_frames = (uint32_t)10;
    float overlap_thresh = (float)0.1;

#endif // DEBUG_BUILD

    //const char *model = "../networks/SSD-Mobilenet-v2/ssd_mobilenet_v2_coco.uff";
    //const char *class_labels = "../networks/SSD-Mobilenet-v2/ssd_coco_labels.txt";
    const char *model = "../networks/SSD-Inception-v2/ssd_inception_v2_coco.uff";
    const char *class_labels = "../networks/SSD-Inception-v2/ssd_coco_labels.txt";
    float thresh = (float)0.5;
    const char *input_blob = "Input";
    const char *output_blob = "NMS"; // for SSD
    //const char *output_blob = "MarkOutput_0"; // for yolo?
    Dims3 inputDims(3, 720, 1280);
    const char *output_count = "NMS_1";

    net = detectNet::Create("SSD_Inception_V2", detection_thresh, max_batch_size); // use downloaded model preloaded with jetson inference
    // net = detectNet::Create("SSD_Mobilenet_V2", detection_thresh, max_batch_size); // use downloaded model preloaded with jetson inference
    //net = detectNet::Create(model, class_labels, thresh, input_blob, inputDims, output_blob, output_count); // load model from specific path
    net->SetTracker(objectTrackerIOU::Create(min_frames, drop_frames, overlap_thresh));

    if (!net)
    {
        LogError("detectnet:  failed to load detectNet model\n");
        return false;
    }

    return true;
}

/********************************************************************************
 * Function: detect_objects
 * Description: Initialize the network used for object detection.
 ********************************************************************************/
void detect_objects(void)
{
    uint32_t overlay_flags = 0;

    overlay_flags = overlay_flags | detectNet::OVERLAY_LABEL | detectNet::OVERLAY_CONFIDENCE | detectNet::OVERLAY_TRACKING | detectNet::OVERLAY_LINES;

    if (overlay_flags > 0 && image != NULL)
    {
        detection_count = net->Detect(image, input->GetWidth(), input->GetHeight(), &detections, overlay_flags);
    }
    else if (image != NULL)
    {
        detection_count = net->Detect(image, input->GetWidth(), input->GetHeight(), &detections);
    }
    else
    {
        // No other options
    }
}
    
/********************************************************************************
 * Function: SSD
 * Description: Class constructor
 ********************************************************************************/
SSD::SSD(void) {};

/********************************************************************************
 * Function: ~SSD
 * Description: Class destructor
 ********************************************************************************/
SSD::~SSD(void) {};

/********************************************************************************
 * Function: initialize_detection_net
 * Description: Delete detection network to free up resources.
 ********************************************************************************/
bool SSD::init(void)
{
    net = NULL;
    detections = NULL;
    detection_count = 0;

    if (!create_detection_network())
    {
        Print::c_fprintf("Failed to create detection network");
        return false;
    }

    return true;
}

/********************************************************************************
 * Function: loop
 * Description: Process video stream and output detected objects.
 ********************************************************************************/
void SSD::loop(void)
{
    detect_objects();
}

/********************************************************************************
 * Function: shutdown
 * Description: Shutdown detection network
 ********************************************************************************/
void SSD::shutdown(void)
{
    LogVerbose("detectnet:  shutting down...\n");
    SAFE_DELETE(net);
    LogVerbose("detectnet:  shutdown complete.\n");
}

#endif // BLD_JETSON_B01
#endif // ENABLE_CV
