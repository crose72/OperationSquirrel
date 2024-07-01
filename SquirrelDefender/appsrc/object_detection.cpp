#ifdef USE_JETSON

/********************************************************************************
 * @file    target_tracking.cpp
 * @author  Cameron Rose
 * @date    6/7/2023
 * @brief   All methods needed to initialize and create a detection network and
            choose a target.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "object_detection.h"

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
int numDetections;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: Detection
 * Description: Class constructor
 ********************************************************************************/
Detection::Detection(void){};

/********************************************************************************
 * Function: ~Detection
 * Description: Class destructor
 ********************************************************************************/
Detection::~Detection(void){};

/********************************************************************************
 * Function: create_detection_network
 * Description: Initialize the network used for object detection.
 ********************************************************************************/
bool Detection::create_detection_network(void)
{
    Parameters detection_params("../params.json");

    float detection_thresh = detection_params.get_float_param("Detection_tracking", "Detect_Thresh");
    uint32_t max_batch_size = detection_params.get_uint32_param("Detection_tracking", "Max_Batch_Size");
    uint32_t min_frames = detection_params.get_uint32_param("Detection_tracking", "Min_Frames");
    uint32_t drop_frames = detection_params.get_uint32_param("Detection_tracking", "Drop_Frames");
    float overlap_thresh = detection_params.get_float_param("Detection_tracking", "Overlap_Threshold");

    const char *model = "../networks/SSD-Mobilenet-v2/ssd_mobilenet_v2_coco.uff";
    const char *class_labels = "../networks/SSD-Mobilenet-v2/ssd_coco_labels.txt";
    float thresh = (float)0.5;
    const char *input_blob = "Input";
    const char *output_blob = "NMS";
    Dims3 inputDims(3, 720, 1280);
    const char *output_count = "NMS_1";

    // net = detectNet::Create("SSD_Inception_V2", detection_thresh, max_batch_size);
    // net = detectNet::Create("SSD_Mobilenet_V2", detection_thresh, max_batch_size);
    net = detectNet::Create(model, class_labels, thresh, input_blob, inputDims, output_blob, output_count);
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
void Detection::detect_objects(void)
{
    uint32_t overlay_flags = 0;

    overlay_flags = overlay_flags | detectNet::OVERLAY_LABEL | detectNet::OVERLAY_CONFIDENCE | detectNet::OVERLAY_TRACKING | detectNet::OVERLAY_LINES;

    if (overlay_flags > 0 && image != NULL)
    {
        numDetections = net->Detect(image, input->GetWidth(), input->GetHeight(), &detections, overlay_flags);
    }
    else if (image != NULL)
    {
        numDetections = net->Detect(image, input->GetWidth(), input->GetHeight(), &detections);
    }
    else
    {
        // No other options
    }
}

/********************************************************************************
 * Function: get_object_info
 * Description: Obtain info about detected objects.
 ********************************************************************************/
void Detection::get_object_info(void)
{
    if (numDetections > 0)
    {
        // LogVerbose("%i objects detected\n", numDetections);

        for (int n = 0; n < numDetections; n++)
        {
            float boxWidth = detections[n].Width();
            float boxHeight = detections[n].Height();
        }
    }
}

/********************************************************************************
 * Function: print_object_info
 * Description: Print info about detected objects.
 ********************************************************************************/
void Detection::print_object_info(void)
{
    if (numDetections > 0)
    {
        LogVerbose("%i objects detected\n", numDetections);

        for (int n = 0; n < numDetections; n++)
        {
            if (detections[n].TrackID >= 0) // is this a tracked object?
            {
                if (detections[n].ClassID == 1 && detections[n].Confidence > 0.5)
                {
                    LogVerbose("\ndetected obj %i  class #%u (%s)  confidence=%f\n", n, detections[n].ClassID, net->GetClassDesc(detections[n].ClassID), detections[n].Confidence);
                    LogVerbose("bounding box %i  (%.2f, %.2f)  (%.2f, %.2f)  w=%.2f  h=%.2f\n", n, detections[n].Left, detections[n].Top, detections[n].Right, detections[n].Bottom, detections[n].Width(), detections[n].Height());
                    LogVerbose("tracking  ID %i  status=%i  frames=%i  lost=%i\n", detections[n].TrackID, detections[n].TrackStatus, detections[n].TrackFrames, detections[n].TrackLost);
                    LogVerbose("Object %i Edges (Left,Right,Top,Bottom)=(%.2f, %.2f, %.2f, %.2f)\n", n, detections[n].Left, detections[n].Right, detections[n].Top, detections[n].Bottom);
                    LogVerbose("video width, video height: (%.2f, %.2f)\n", input_video_width, input_video_height);
                    LogVerbose("box width, box height: (%.2f, %.2f)\n", detections[n].Width(), detections[n].Height());
                }
            }
        }
    }
}

/********************************************************************************
 * Function: print_print_performance_statsobject_info
 * Description: Print info about detection network performance.
 ********************************************************************************/
void Detection::print_performance_stats(void)
{
    net->PrintProfilerTimes();
}

/********************************************************************************
 * Function: delete_tracking_net
 * Description: Delete detection network to free up resources.
 ********************************************************************************/
void Detection::delete_tracking_net(void)
{
    SAFE_DELETE(net);
}

/********************************************************************************
 * Function: usage
 * Description: Display a help message.
 ********************************************************************************/
int Detection::print_usage(void)
{
    PrintPass::c_printf("usage: detectnet [--help] [--network=NETWORK] [--threshold=THRESHOLD] ...\n");
    PrintPass::c_printf("                 input [output]\n\n");
    PrintPass::c_printf("Locate objects in a video/image stream using an object detection DNN.\n");
    PrintPass::c_printf("See below for additional arguments that may not be shown above.\n\n");
    PrintPass::c_printf("positional arguments:\n");
    PrintPass::c_printf("    input           resource URI of input stream  (see videoSource below)\n");
    PrintPass::c_printf("    output          resource URI of output stream (see videoOutput below)\n\n");

    PrintPass::c_printf("%s", detectNet::Usage());
    PrintPass::c_printf("%s", objectTracker::Usage());
    PrintPass::c_printf("%s", videoSource::Usage());
    PrintPass::c_printf("%s", videoOutput::Usage());
    PrintPass::c_printf("%s", Log::Usage());

    return 0;
}

/********************************************************************************
 * Function: detection_loop
 * Description: Process video stream and output detected objects.
 ********************************************************************************/
void Detection::detection_loop(void)
{
    detect_objects();
    get_object_info();
}

/********************************************************************************
 * Function: initialize_detection_net
 * Description: Delete detection network to free up resources.
 ********************************************************************************/
bool Detection::detection_net_init(void)
{
    net = NULL;
    detections = NULL;
    numDetections = 0;

    if (!create_detection_network())
    {
        PrintPass::c_fprintf("Failed to create detection network");
        return false;
    }

    return true;
}

/********************************************************************************
 * Function: shutdown
 * Description: Shutdown detection network
 ********************************************************************************/
void Detection::shutdown(void)
{
    LogVerbose("detectnet:  shutting down...\n");
    Detection::delete_tracking_net();
    LogVerbose("detectnet:  shutdown complete.\n");
}

#endif // USE_JETSON