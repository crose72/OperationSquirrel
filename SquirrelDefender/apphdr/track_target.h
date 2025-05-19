#pragma once

#ifdef ENABLE_CV

/********************************************************************************
 * @file    track_target.h
 * @author  Cameron Rose
 * @date    1/22/2025
 ********************************************************************************/
#ifndef TRACK_TARGET_H
#define TRACK_TARGET_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "video_io.h"
#include "detect_target.h"
#include "param_reader.h"
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/cudawarping.hpp>

#ifdef BLD_JETSON_B01

#include <jetson-utils/cudaMappedMemory.h> // Assuming Jetson Inference utilities are available
#include <jetson-utils/cudaRGB.h>          // For cuda functions

#endif // BLD_JETSON_B01

/********************************************************************************
 * Imported objects
 ********************************************************************************/

#ifdef BLD_JETSON_B01

extern detectNet *g_net;
extern detectNet::Detection *g_detections;
extern uchar3 *g_image;
extern int g_detection_count;
extern float g_input_video_width;
extern float g_input_video_height;

#elif defined(BLD_JETSON_ORIN_NANO)

extern std::vector<Object> g_yolo_detections;
extern int g_yolo_detection_count;
extern cv::Mat g_image;
extern float g_input_video_width;
extern float g_input_video_height;

#elif defined(BLD_WIN)

extern cv::dnn::Net g_net;
extern std::vector<YoloNet::detection> g_yolo_detections;
extern int g_yolo_detection_count;
extern cv::Mat g_image;
extern float g_input_video_width;
extern float g_input_video_height;

#else

#error "Please define a build platform."

#endif // BLD_JETSON_B01

extern bool g_valid_image_rcvd;

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern bool g_target_valid;
extern int g_target_detection_id;
extern int g_target_track_id;
extern float g_target_cntr_offset_x;
extern float g_target_cntr_offset_y;
extern float g_target_height;
extern float g_target_width;
extern float g_target_aspect;
extern float g_target_left;
extern float g_target_right;
extern float g_target_top;
extern float g_target_bottom;
extern float g_target_center_x;
extern float g_target_center_y;
extern float g_detection_class;
extern float g_target_detection_conf;

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class Track
{
public:
    Track();
    ~Track();

    static bool init(void);
    static void loop(void);
    static void shutdown(void);

private:
};

#endif // TRACK_TARGET_H

#endif // ENABLE_CV
