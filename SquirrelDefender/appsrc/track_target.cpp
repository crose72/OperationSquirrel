#ifdef ENABLE_CV

/********************************************************************************
 * @file    track_target.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Maintain bounding box around a detected target, even when object
 *          detection fails.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "track_target.h"
#include "video_io.h"
#include "detect_target.h"
#include "OSNet.h"
#include "YOLOv8.h"
#include "video_io.h"
#include "param_reader.h"
#include "signal_processing.h"
#include <opencv2/opencv.hpp>

#ifdef BLD_JETSON_B01

#include <jetson-utils/cudaMappedMemory.h> // Assuming Jetson Inference utilities are available
#include <jetson-utils/cudaRGB.h>          // For cuda functions

#endif // BLD_JETSON_B01

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/
struct Track
{
    Object obj;
    int id;
};

/********************************************************************************
 * Object definitions
 ********************************************************************************/
bool target_tracked;
bool tracking;
bool initialized_cv_image;
bool initialized_tracker;
float target_detection_thresh;
float target_class;
bool target_valid_prv;
bool g_target_valid;
int g_target_detection_id;
int g_target_track_id;
float g_target_cntr_offset_x;
float g_target_cntr_offset_y;
float g_target_center_y;
float g_target_center_x;
float g_target_height;
float g_target_width;
float g_target_aspect;
float g_target_left;
float g_target_right;
float g_target_top;
float g_target_bottom;
float g_detection_class;
float g_target_detection_conf;
float g_target_cntr_offset_x_filt;
float g_target_cntr_offset_y_filt;
float target_cntr_offset_x_prv;
float target_cntr_offset_y_prv;
cv::Rect target_bounding_box;
std::vector<int> target_candidates;
std::vector<cv::cuda::GpuMat> target_candidate_imgs;
std::vector<cv::Rect> target_candidate_bboxs;
std::vector<std::vector<float>> target_candidate_embeddings; // one per crop
std::vector<std::vector<float>> tracked_object_features;
std::vector<Track> tracked_targets;
OSNet *osnet_extractor;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/
float target_bbox_center_filt_coeff = (float)0.75;
float target_similarity_thresh = (float)0.55;
int max_reid_batch = (int)32;

/********************************************************************************
 * Function definitions
 ********************************************************************************/
void get_tracking_params(void);
void filter_detections(void);
void track_objects(void);
void select_target(void);
void get_target_info(void);
void validate_target(void);
void update_target_info(void);
static float cosineSimilarity(const std::vector<float> &a, const std::vector<float> &b);

// Cosine similarity between two L2-normalized target_candidate_embeddings
static float cosineSimilarity(const std::vector<float> &a, const std::vector<float> &b)
{
    if (a.size() != b.size() || a.empty())
        return 0.0f;
    float s = 0.0f;
    for (size_t i = 0; i < a.size(); ++i)
        s += a[i] * b[i];
    return s;
}

/********************************************************************************
 * Function: get_tracking_params
 * Description: Calibratable parameters for target tracking and identification.
 ********************************************************************************/
void get_tracking_params(void)
{
    ParamReader target_params("../params.json");

    target_bbox_center_filt_coeff = target_params.get_float_param("Tracking_Params", "BBox_Filt_coeff");
    target_detection_thresh = target_params.get_float_param("Tracking_Params", "Detect_Thresh");

#if defined(BLD_JETSON_B01)

    target_class = target_params.get_int_param("Tracking_Params", "Detect_Class_B01");

#elif defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WSL)

    target_class = target_params.get_int_param("Tracking_Params", "Detect_Class_Orin");
    target_similarity_thresh = target_params.get_float_param("Tracking_Params", "Track_similarity_thresh");
    max_reid_batch = target_params.get_int_param("Tracking_Params", "Track_max_reid_batch");

#elif defined(BLD_WIN)

    target_class = target_params.get_int_param("Tracking_Params", "Detect_Class_Orin");

#else

#error "Please define build platform."

#endif
}

/********************************************************************************
 * Function: filter_detections
 * Description: Determine which detected objects to consider for tracking.
 ********************************************************************************/
void filter_detections(void)
{
#if defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WSL)

    target_candidates.clear();
    target_candidate_imgs.clear();
    target_candidate_bboxs.clear();

    target_candidate_imgs.reserve(g_yolo_detection_count);
    target_candidate_bboxs.reserve(g_yolo_detection_count);

    for (int n = 0; n < g_yolo_detection_count; ++n)
    {
        /* A detected object, classified as a person with some confidence level */
        if (g_yolo_detections[n].label == target_class &&
            g_yolo_detections[n].probability > target_detection_thresh)
        {
            cv::Rect bbox = g_yolo_detections[n].rect; // Rect2f -> Rect (int)
            bbox &= cv::Rect(0, 0, g_cam0_image_gpu.cols, g_cam0_image_gpu.rows);

            if (bbox.width <= 0 || bbox.height <= 0)
            {
                continue;
            }

            target_candidates.emplace_back(n);
            target_candidate_bboxs.push_back(bbox);
            target_candidate_imgs.emplace_back(g_cam0_image_gpu(bbox));
        }
    }

#endif
}

/********************************************************************************
 * Function: track_objects
 * Description: Match detections to tracked objects.
 ********************************************************************************/
void track_objects(void)
{
#if defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WSL)

    // Extract features of each detected target
    if (!target_candidate_imgs.empty() && target_candidate_imgs.size() <= max_reid_batch)
    {
        target_candidate_embeddings = osnet_extractor->extractFeatures(target_candidate_imgs);
    };

    // “match-or-add” using cosine similarity
    for (size_t i = 0; i < target_candidate_embeddings.size(); ++i)
    {
        // 512-D vector (L2-normalized by OSNet)
        const std::vector<float> &f_vec = target_candidate_embeddings[i];

        int best_idx = -1;
        float best_s = -std::numeric_limits<float>::infinity();

        // Calculate similarity between a tracked object
        // and all of the currently detected objects to find the
        // detection that most closely matches the tracked object
        for (size_t j = 0; j < tracked_object_features.size(); ++j)
        {
            float s = cosineSimilarity(f_vec, tracked_object_features[j]);

            if (s > best_s)
            {
                best_s = s;
                best_idx = static_cast<int>(j);
            }
        }

        // if a match is found (one of the detected objects
        // matches a previously tracked object) then update
        // the feature vectore of the tracked object with the
        // feature vector of the detection which most closely matches
        if (best_idx >= 0 && best_s >= target_similarity_thresh)
        {
            // replace (or EMA-average if you want smoother updates)
            tracked_object_features[best_idx] = f_vec;
        }
        else
        {
            // Add a detection to tracked objects since because no
            // match was found
            tracked_object_features.push_back(f_vec);
            best_idx = static_cast<int>(tracked_object_features.size() - 1);
        }

        // Draw on CPU image
        // const cv::Rect &r = target_candidate_bboxs[i];
        // cv::rectangle(g_cam0_image, r, cv::Scalar(0, 255, 0), 2);
        // char txt[128];
        // std::snprintf(txt, sizeof(txt), "id=%d s=%.2f", best_idx, best_s);
        // cv::putText(g_cam0_image, txt, {r.x, std::max(0, r.y - 5)},
        //             cv::FONT_HERSHEY_SIMPLEX, 0.5, {255, 255, 255}, 1);
    }

#endif
}

/********************************************************************************
 * Function: select_target
 * Description: Select the target to follow from one of the tracked objects.
 ********************************************************************************/
void select_target(void)
{
    g_target_detection_id = -1;

#if defined(BLD_JETSON_B01)

    for (int n = 0; n < g_detection_count; ++n)
    {
        /* A tracked object, classified as a person with some confidence level */
        if (g_detections[n].TrackID >= 0 &&
            g_detections[n].ClassID == target_class &&
            g_detections[n].Confidence > target_detection_thresh)
        {
            g_detection_class = g_detections[n].ClassID;
            g_target_detection_conf = g_detections[n].Confidence;
            g_target_detection_id = n;
            return;
        }
    }

#elif defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WSL)

    for (int n = 0; n < g_yolo_detection_count; ++n)
    {
        /* A tracked object, classified as a person with some confidence level */
        if (g_yolo_detections[n].label == target_class &&
            g_yolo_detections[n].probability > target_detection_thresh)
        {
            g_detection_class = g_yolo_detections[n].label;
            g_target_detection_conf = g_yolo_detections[n].probability;
            g_target_detection_id = n;
            return;
        }
    }

#elif defined(BLD_WIN)

    for (int n = 0; n < g_yolo_detection_count; ++n)
    {
        /* A tracked object, classified as a person with some confidence level */
        if (g_yolo_detections[n].ClassID == target_class &&
            g_yolo_detections[n].Confidence > target_detection_thresh)
        {
            g_detection_class = g_yolo_detections[n].ClassID;
            g_target_detection_conf = g_yolo_detections[n].Confidence;
            g_target_detection_id = n;
            return;
        }
    }

#else

#error "Please define build platform."

#endif
}

/********************************************************************************
 * Function: get_target_info
 * Description: Obtain the important information about the target, such as the
 *              height and location of the center of the target in the frame.
 ********************************************************************************/
void get_target_info(void)
{
    g_target_track_id = (int)-1;

#if defined(BLD_JETSON_B01)

    if (g_target_detection_id >= 0)
    {
        g_target_height = g_detections[g_target_detection_id].Height();
        g_target_width = g_detections[g_target_detection_id].Width();
        g_target_track_id = g_detections[g_target_detection_id].TrackID;
        g_target_left = g_detections[g_target_detection_id].Left;
        g_target_right = g_detections[g_target_detection_id].Right;
        g_target_top = g_detections[g_target_detection_id].Top;
        g_target_bottom = g_detections[g_target_detection_id].Bottom;
    }

#elif defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WSL)

    if (g_target_detection_id >= 0)
    {
        g_target_height = g_yolo_detections[g_target_detection_id].rect.height;
        g_target_width = g_yolo_detections[g_target_detection_id].rect.width;
        g_target_track_id = 0;
        g_target_left = g_yolo_detections[g_target_detection_id].rect.x;
        g_target_right = g_target_left + g_target_width;
        g_target_top = g_yolo_detections[g_target_detection_id].rect.y;
    }

#elif defined(BLD_WIN)

    if (g_target_detection_id >= 0)
    {
        g_target_height = g_yolo_detections[g_target_detection_id].rect.height;
        g_target_width = g_yolo_detections[g_target_detection_id].rect.width;
        g_target_track_id = 0;
        g_target_left = g_yolo_detections[g_target_detection_id].rect.x;
        g_target_right = g_target_left + g_target_width;
        g_target_top = g_yolo_detections[g_target_detection_id].rect.y;
    }

#else

#error "Please define a build platform."

#endif

    g_target_bottom = g_target_top + g_target_height;
    g_target_center_y = (g_target_left + g_target_right) / 2.0f;
    g_target_center_x = (g_target_bottom + g_target_top) / 2.0f;
    g_target_cntr_offset_y = g_target_center_y - g_cam0_video_width_center;
    g_target_cntr_offset_x = g_target_center_x - g_cam0_video_height_center;
    g_target_aspect = g_target_width / g_target_height;

    g_target_cntr_offset_x_filt = low_pass_filter(g_target_cntr_offset_x, target_cntr_offset_x_prv, target_bbox_center_filt_coeff);
    g_target_cntr_offset_y_filt = low_pass_filter(g_target_cntr_offset_y, target_cntr_offset_y_prv, target_bbox_center_filt_coeff);
}

/********************************************************************************
 * Function: validate_target
 * Description: Determine which detected object to track.
 ********************************************************************************/
void validate_target(void)
{
    /* Target detected, tracked, and has a size greater than 0.  Controls based on the target may be
    implimented. */
    if (g_target_detection_id >= 0 && g_target_track_id >= 0 && g_target_height > 1 && g_target_width > 1)
    {
        g_target_valid = true;
    }
    else
    {
        g_target_valid = false;
    }

    target_valid_prv = g_target_valid;
}

/********************************************************************************
 * Function: update_target_info
 * Description: Obtain the important information about the target, such as the
 *              height and location of the center of the target in the frame.
 ********************************************************************************/
void update_target_info(void)
{
    g_target_height = target_bounding_box.height;
    g_target_width = target_bounding_box.width;
    g_target_left = target_bounding_box.x;
    g_target_right = g_target_left + g_target_width;
    g_target_top = target_bounding_box.y;
    g_target_bottom = g_target_top + g_target_height;
    g_target_center_y = (g_target_left + g_target_right) / 2.0f;
    g_target_center_x = (g_target_bottom + g_target_top) / 2.0f;
    g_target_cntr_offset_y = g_target_center_y - g_cam0_video_width_center;
    g_target_cntr_offset_x = g_target_center_x - g_cam0_video_height_center;
    g_target_aspect = (g_target_height >= 0.0000001 ? g_target_width / g_target_height : (float)0.0);
}

/********************************************************************************
 * Function: Tracking
 * Description: Tracking class constructor.
 ********************************************************************************/
Tracking::Tracking(void) {};

/********************************************************************************
 * Function: ~Tracking
 * Description: Tracking class destructor.
 ********************************************************************************/
Tracking::~Tracking(void) {};

/********************************************************************************
 * Function: init
 * Description: Initialize all track target variables.  Run once at the start
 *              of the program.
 ********************************************************************************/
bool Tracking::init(void)
{
    get_tracking_params();

    g_target_valid = false;
    target_valid_prv = false;
    g_target_cntr_offset_x = (float)0.0;
    g_target_cntr_offset_y = (float)0.0;
    g_target_height = (float)0.0;
    g_target_width = (float)0.0;
    g_target_aspect = (float)0.0;
    g_target_left = (float)0.0;
    g_target_right = (float)0.0;
    g_target_top = (float)0.0;
    g_target_bottom = (float)0.0;
    g_target_detection_id = -1;
    g_detection_class = (float)0.0;
    g_target_detection_conf = (float)0.0;

    g_target_cntr_offset_x_filt = (float)0.0;
    g_target_cntr_offset_y_filt = (float)0.0;
    target_cntr_offset_x_prv = (float)0.0;
    target_cntr_offset_y_prv = (float)0.0;

    initialized_cv_image = false;
    target_tracked = false;
    tracking = false;
    target_candidates.clear();

#if defined(BLD_WSL)

    OSNet::Config config;
    const std::string engine_path =
        "/workspace/OperationSquirrel/SquirrelDefender/networks/osnet/oxnet_x0_25.engine.NVIDIAGeForceRTX3060LaptopGPU.fp16.batch32";

    osnet_extractor = new OSNet(engine_path, config);

#elif defined(BLD_JETSON_ORIN_NANO)

    OSNet::Config config;
    const std::string engine_path =
        "/workspace/OperationSquirrel/SquirrelDefender/networks/osnet/osnet_x0_25.engine.Orin.fp16.batch32";

    osnet_extractor = new OSNet(engine_path, config);

#endif

    return true;
}

/********************************************************************************
 * Function: loop
 * Description: Determine target to be tracked and maintain identity of target
 *              from loop to loop.
 ********************************************************************************/
void Tracking::loop(void)
{
    filter_detections();
    track_objects();
    select_target();
    get_target_info();
    validate_target();
}

/********************************************************************************
 * Function: shutdown
 * Description: Cleanup code to run at the end of the program.
 ********************************************************************************/
void Tracking::shutdown(void)
{
}

#endif // ENABLE_CV
