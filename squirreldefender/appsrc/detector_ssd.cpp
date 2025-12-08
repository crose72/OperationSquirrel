#ifdef ENABLE_CV
#ifdef BLD_JETSON_B01

/********************************************************************************
 * @file    detector_ssd.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   This module initializes and manages the SSD Mobilenet object
 *          detector using the Jetson-Inference library. It performs object
 *          detection on incoming camera frames and provides a simple
 *          init/loop/shutdown interface for the embedded system.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "jetson-utils/videoSource.h"
#include "jetson-utils/videoOutput.h"
#include "jetson-inference/objectTracker.h"
#include <jetson-inference/objectTrackerIOU.h>
#include <jetson-inference/objectTrackerKLT.h>

#include "common_inc.h"
#include "detector_ssd.h"
#include "video_io.h"
#include "param_reader.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
detectNet *g_det_nv_net = nullptr;
detectNet::Detection *g_det_nv_list = nullptr;
int g_det_nv_count = (int)0;

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
 * Description: Initialize the Jetson-Inference detectNet + tracker.
 ********************************************************************************/
bool create_detection_network()
{
#ifdef DEBUG_BUILD
    ParamReader params("../params.json");

    const float detection_thresh = params.get_float_param("target_det_params.target_det_conf_thresh");
    const uint32_t max_batch_size = params.get_uint32_param("target_det_params.max_batch_size");
    const uint32_t min_frames = params.get_uint32_param("target_det_params.min_hits_to_track");
    const uint32_t drop_frames = params.get_uint32_param("target_det_params.min_frames_to_drop_track");
    const float overlap_thresh = params.get_float_param("target_det_params.track_iou_thresh");
#else
    const float detection_thresh = 0.5f;
    const uint32_t max_batch_size = 4;
    const uint32_t min_frames = 25;
    const uint32_t drop_frames = 10;
    const float overlap_thresh = 0.1f;
#endif

    // Network files
    const char *model_path = "../networks/DetectorSSD-Mobilenet-v2/ssd_mobilenet_v2_coco.uff";
    const char *label_path = "../networks/DetectorSSD-Mobilenet-v2/ssd_coco_labels.txt";

    // Input/output layers for DetectorSSD models
    const char *input_blob = "Input";
    const char *output_blob = "NMS";
    const char *output_count = "NMS_1";
    Dims3 input_dims(3, 720, 1280);

    // Create network
    g_det_nv_net = detectNet::Create(model_path, label_path, detection_thresh,
                                     input_blob, input_dims, output_blob, output_count);

    if (!g_det_nv_net)
    {
        spdlog::error("detectNet: Failed to load detection model.");
        return false;
    }

    // Create and attach IOU tracker
    auto tracker = objectTrackerIOU::Create(min_frames, drop_frames, overlap_thresh);
    if (!tracker)
    {
        spdlog::warn("detectNet: Tracker creation failed. Continuing without tracking.");
    }
    else
    {
        g_det_nv_net->SetTracker(tracker);
    }

    return true;
}

/********************************************************************************
 * Function: detect_targets
 * Description: Runs detection on the current camera frame (CPU pointer).
 ********************************************************************************/
void detect_targets()
{
    if (!g_det_nv_net || !g_cam0_img_cpu || !g_input)
        return;

    const uint32_t width = g_input->GetWidth();
    const uint32_t height = g_input->GetHeight();

    const uint32_t overlay_flags =
        detectNet::OVERLAY_LABEL |
        detectNet::OVERLAY_CONFIDENCE |
        detectNet::OVERLAY_TRACKING |
        detectNet::OVERLAY_LINES;

    g_det_nv_count = g_det_nv_net->Detect(g_cam0_img_cpu, width, height,
                                          &g_det_nv_list, overlay_flags);
}

/********************************************************************************
 * Function: DetectorSSD
 * Description: Class constructor
 ********************************************************************************/
DetectorSSD::DetectorSSD() {}

/********************************************************************************
 * Function: ~DetectorSSD
 * Description: Class destructor
 ********************************************************************************/
DetectorSSD::~DetectorSSD() {}

/********************************************************************************
 * Function: init
 * Description: Initialize detection network.
 ********************************************************************************/
bool DetectorSSD::init(void)
{
    if (!create_detection_network())
    {
        spdlog::error("Failed to create detection network");
        return false;
    }

    return true;
}

/********************************************************************************
 * Function: loop
 * Description: Process video stream and output detected objects.
 ********************************************************************************/
void DetectorSSD::loop(void)
{
    detect_targets();
}

/********************************************************************************
 * Function: shutdown
 * Description: Shutdown detection network
 ********************************************************************************/
void DetectorSSD::shutdown(void)
{
    LogVerbose("detectnet:  shutting down...\n");
    // Jetson-inference detector uses raw pointer ownership → delete explicitly
    SAFE_DELETE(g_det_nv_net);
    LogVerbose("detectnet:  shutdown complete.\n");
}

#endif // BLD_JETSON_B01
#endif // ENABLE_CV
