#ifdef ENABLE_CV

/********************************************************************************
 * @file    target_tracking.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Maintain bounding box around a detected target, even when object
 *          detection fails.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "target_tracking.h"
#include "video_io.h"
#include "target_detection.h"
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
bool g_tgt_valid;
int g_tgt_detect_id;
int g_tgt_track_id;
float g_tgt_cntr_offset_y_pix;
float g_tgt_cntr_offset_x_pix;
float g_tgt_center_x_px;
float g_tgt_center_y_px;
float g_tgt_height_pix;
float g_tgt_width_pix;
float g_tgt_aspect_ratio;
float g_tgt_left_px;
float g_tgt_right_px;
float g_tgt_top_px;
float g_tgt_bottom_px;
float g_tgt_class_id;
float g_tgt_conf;
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

    target_candidate_imgs.reserve(g_det_count);
    target_candidate_bboxs.reserve(g_det_count);

    for (int n = 0; n < g_det_count; ++n)
    {
        /* A detected object, classified as a person with some confidence level */
        if (g_det_yolo_list[n].label == target_class &&
            g_det_yolo_list[n].probability > target_detection_thresh)
        {
            cv::Rect bbox = g_det_yolo_list[n].rect; // Rect2f -> Rect (int)
            bbox &= cv::Rect(0, 0, g_cam0_img_gpu.cols, g_cam0_img_gpu.rows);

            if (bbox.width <= 0 || bbox.height <= 0)
            {
                continue;
            }

            target_candidates.emplace_back(n);
            target_candidate_bboxs.push_back(bbox);
            target_candidate_imgs.emplace_back(g_cam0_img_gpu(bbox));
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
        // cv::rectangle(g_cam0_img_cpu, r, cv::Scalar(0, 255, 0), 2);
        // char txt[128];
        // std::snprintf(txt, sizeof(txt), "id=%d s=%.2f", best_idx, best_s);
        // cv::putText(g_cam0_img_cpu, txt, {r.x, std::max(0, r.y - 5)},
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
    g_tgt_detect_id = -1;

#if defined(BLD_JETSON_B01)

    for (int n = 0; n < g_det_nv_count; ++n)
    {
        /* A tracked object, classified as a person with some confidence level */
        if (g_det_nv_list[n].TrackID >= 0 &&
            g_det_nv_list[n].ClassID == target_class &&
            g_det_nv_list[n].Confidence > target_detection_thresh)
        {
            g_tgt_class_id = g_det_nv_list[n].ClassID;
            g_tgt_conf = g_det_nv_list[n].Confidence;
            g_tgt_detect_id = n;
            return;
        }
    }

#elif defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WSL)

    for (int n = 0; n < g_det_count; ++n)
    {
        /* A tracked object, classified as a person with some confidence level */
        if (g_det_yolo_list[n].label == target_class &&
            g_det_yolo_list[n].probability > target_detection_thresh)
        {
            g_tgt_class_id = g_det_yolo_list[n].label;
            g_tgt_conf = g_det_yolo_list[n].probability;
            g_tgt_detect_id = n;
            return;
        }
    }

#elif defined(BLD_WIN)

    for (int n = 0; n < g_det_count; ++n)
    {
        /* A tracked object, classified as a person with some confidence level */
        if (g_det_yolo_list[n].ClassID == target_class &&
            g_det_yolo_list[n].Confidence > target_detection_thresh)
        {
            g_tgt_class_id = g_det_yolo_list[n].ClassID;
            g_tgt_conf = g_det_yolo_list[n].Confidence;
            g_tgt_detect_id = n;
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
    g_tgt_track_id = (int)-1;

#if defined(BLD_JETSON_B01)

    if (g_tgt_detect_id >= 0)
    {
        g_tgt_height_pix = g_det_nv_list[g_tgt_detect_id].Height();
        g_tgt_width_pix = g_det_nv_list[g_tgt_detect_id].Width();
        g_tgt_track_id = g_det_nv_list[g_tgt_detect_id].TrackID;
        g_tgt_left_px = g_det_nv_list[g_tgt_detect_id].Left;
        g_tgt_right_px = g_det_nv_list[g_tgt_detect_id].Right;
        g_tgt_top_px = g_det_nv_list[g_tgt_detect_id].Top;
        g_tgt_bottom_px = g_det_nv_list[g_tgt_detect_id].Bottom;
    }

#elif defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WSL)

    if (g_tgt_detect_id >= 0)
    {
        g_tgt_height_pix = g_det_yolo_list[g_tgt_detect_id].rect.height;
        g_tgt_width_pix = g_det_yolo_list[g_tgt_detect_id].rect.width;
        g_tgt_track_id = 0;
        g_tgt_left_px = g_det_yolo_list[g_tgt_detect_id].rect.x;
        g_tgt_right_px = g_tgt_left_px + g_tgt_width_pix;
        g_tgt_top_px = g_det_yolo_list[g_tgt_detect_id].rect.y;
    }

#elif defined(BLD_WIN)

    if (g_tgt_detect_id >= 0)
    {
        g_tgt_height_pix = g_det_yolo_list[g_tgt_detect_id].rect.height;
        g_tgt_width_pix = g_det_yolo_list[g_tgt_detect_id].rect.width;
        g_tgt_track_id = 0;
        g_tgt_left_px = g_det_yolo_list[g_tgt_detect_id].rect.x;
        g_tgt_right_px = g_tgt_left_px + g_tgt_width_pix;
        g_tgt_top_px = g_det_yolo_list[g_tgt_detect_id].rect.y;
    }

#else

#error "Please define a build platform."

#endif

    g_tgt_bottom_px = g_tgt_top_px + g_tgt_height_pix;
    g_tgt_center_x_px = (g_tgt_left_px + g_tgt_right_px) / 2.0f;
    g_tgt_center_y_px = (g_tgt_bottom_px + g_tgt_top_px) / 2.0f;
    g_tgt_cntr_offset_x_pix = g_tgt_center_x_px - g_cam0_img_width_cx;
    g_tgt_cntr_offset_y_pix = g_tgt_center_y_px - g_cam0_img_height_cy;
    g_tgt_aspect_ratio = g_tgt_width_pix / g_tgt_height_pix;
}

/********************************************************************************
 * Function: validate_target
 * Description: Determine which detected object to track.
 ********************************************************************************/
void validate_target(void)
{
    /* Target detected, tracked, and has a size greater than 0.  Controls based on the target may be
    implimented. */
    if (g_tgt_detect_id >= 0 && g_tgt_track_id >= 0 && g_tgt_height_pix > 1 && g_tgt_width_pix > 1)
    {
        g_tgt_valid = true;
    }
    else
    {
        g_tgt_valid = false;
    }

    target_valid_prv = g_tgt_valid;
}

/********************************************************************************
 * Function: update_target_info
 * Description: Obtain the important information about the target, such as the
 *              height and location of the center of the target in the frame.
 ********************************************************************************/
void update_target_info(void)
{
    g_tgt_height_pix = target_bounding_box.height;
    g_tgt_width_pix = target_bounding_box.width;
    g_tgt_left_px = target_bounding_box.x;
    g_tgt_right_px = g_tgt_left_px + g_tgt_width_pix;
    g_tgt_top_px = target_bounding_box.y;
    g_tgt_bottom_px = g_tgt_top_px + g_tgt_height_pix;
    g_tgt_center_x_px = (g_tgt_left_px + g_tgt_right_px) / 2.0f;
    g_tgt_center_y_px = (g_tgt_bottom_px + g_tgt_top_px) / 2.0f;
    g_tgt_cntr_offset_x_pix = g_tgt_center_x_px - g_cam0_img_width_cx;
    g_tgt_cntr_offset_y_pix = g_tgt_center_y_px - g_cam0_img_height_cy;
    g_tgt_aspect_ratio = (g_tgt_height_pix >= 0.0000001 ? g_tgt_width_pix / g_tgt_height_pix : (float)0.0);
}

/********************************************************************************
 * Function: TargetTracking
 * Description: TargetTracking class constructor.
 ********************************************************************************/
TargetTracking::TargetTracking(void) {};

/********************************************************************************
 * Function: ~TargetTracking
 * Description: TargetTracking class destructor.
 ********************************************************************************/
TargetTracking::~TargetTracking(void) {};

/********************************************************************************
 * Function: init
 * Description: Initialize all track target variables.  Run once at the start
 *              of the program.
 ********************************************************************************/
bool TargetTracking::init(void)
{
    get_tracking_params();

    g_tgt_valid = false;
    target_valid_prv = false;
    g_tgt_cntr_offset_y_pix = (float)0.0;
    g_tgt_cntr_offset_x_pix = (float)0.0;
    g_tgt_height_pix = (float)0.0;
    g_tgt_width_pix = (float)0.0;
    g_tgt_aspect_ratio = (float)0.0;
    g_tgt_left_px = (float)0.0;
    g_tgt_right_px = (float)0.0;
    g_tgt_top_px = (float)0.0;
    g_tgt_bottom_px = (float)0.0;
    g_tgt_detect_id = -1;
    g_tgt_class_id = (float)0.0;
    g_tgt_conf = (float)0.0;

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
void TargetTracking::loop(void)
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
void TargetTracking::shutdown(void)
{
}

#endif // ENABLE_CV
