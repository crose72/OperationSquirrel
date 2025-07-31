#ifdef ENABLE_CV
#ifdef BLD_JETSON_B01

/********************************************************************************
 * @file    detect_target_nv.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   This file contains the functions to initialize, run, and clean up
            object detection code.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "detect_target_nv.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
detectNet *g_net;
detectNet::Detection *g_detections;
int g_detection_count;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/
bool create_detection_network(void);
void detect_targets(void);

/********************************************************************************
 * Function: create_detection_network
 * Description: Initialize the network used for object detection.
 ********************************************************************************/
bool create_detection_network(void)
{
#ifdef DEBUG_BUILD

    detect_target_nv.h detection_params("../params.json");

    float detection_thresh = detection_params.get_float_param("Detection_Params", "Detect_Thresh");
    uint32_t max_batch_size = detection_params.get_uint32_param("Detection_Params", "Max_Batch_Size");
    uint32_t min_frames = detection_params.get_uint32_param("Detection_Params", "Min_Frames");
    uint32_t drop_frames = detection_params.get_uint32_param("Detection_Params", "Drop_Frames");
    float overlap_thresh = detection_params.get_float_param("Detection_Params", "Overlap_Threshold");

#else

    float detection_thresh = (float)0.5;
    uint32_t max_batch_size = (uint32_t)4;
    uint32_t min_frames = (uint32_t)25;
    uint32_t drop_frames = (uint32_t)10;
    float overlap_thresh = (float)0.1;

#endif // DEBUG_BUILD

    const char *model = "../models/SSD-Mobilenet-v2/ssd_mobilenet_v2_coco.uff";
    const char *class_labels = "../models/SSD-Mobilenet-v2/ssd_coco_labels.txt";
    // const char *model = "../models/SSD-Inception-v2/ssd_inception_v2_coco.uff";
    // const char *class_labels = "../models/SSD-Inception-v2/ssd_coco_labels.txt";
    float thresh = (float)0.5;
    const char *input_blob = "Input";
    const char *output_blob = "NMS"; // for SSD
    // const char *output_blob = "MarkOutput_0"; // for yolo?
    Dims3 inputDims(3, 720, 1280);
    const char *output_count = "NMS_1";

    // g_net = detectNet::Create("SSD_Inception_V2", detection_thresh, max_batch_size); // use downloaded model preloaded with jetson inference
    // g_net = detectNet::Create("SSD_Mobilenet_V2", detection_thresh, max_batch_size); // use downloaded model preloaded with jetson inference
    g_net = detectNet::Create(model, class_labels, thresh, input_blob, inputDims, output_blob, output_count); // load model from specific path
    g_net->SetTracker(objectTrackerIOU::Create(min_frames, drop_frames, overlap_thresh));

    if (!g_net)
    {
        LogError("detectnet:  failed to load detectNet model\n");
        return false;
    }

    return true;
}

/********************************************************************************
 * Function: detect_targets
 * Description: Run object detection on image and return the detections and
 *              container of detected objects.
 ********************************************************************************/
void detect_targets(void)
{
    uint32_t overlay_flags = 0;

    overlay_flags = overlay_flags | detectNet::OVERLAY_LABEL | detectNet::OVERLAY_CONFIDENCE | detectNet::OVERLAY_TRACKING | detectNet::OVERLAY_LINES;

    if (overlay_flags > 0 && g_image != NULL)
    {
        g_detection_count = g_net->Detect(g_image, g_input->GetWidth(), g_input->GetHeight(), &g_detections, overlay_flags);
    }
    else if (g_image != NULL)
    {
        g_detection_count = g_net->Detect(g_image, g_input->GetWidth(), g_input->GetHeight(), &g_detections);
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
    g_net = NULL;
    g_detections = NULL;
    g_detection_count = 0;

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
    detect_targets();
}

/********************************************************************************
 * Function: shutdown
 * Description: Shutdown detection network
 ********************************************************************************/
void SSD::shutdown(void)
{
    LogVerbose("detectnet:  shutting down...\n");
    SAFE_DELETE(g_net);
    LogVerbose("detectnet:  shutdown complete.\n");
}

#endif // BLD_JETSON_B01
#endif // ENABLE_CV
