#pragma once

#ifdef ENABLE_CV

/********************************************************************************
 * @file    track_target.h
 * @author  Cameron Rose
 * @date    6/7/2023
 ********************************************************************************/
#ifndef TRACK_TARGET_H
#define TRACK_TARGET_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "video_io.h"
#include "detect_target.h"
#include "json_utils.h"
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

extern detectNet *net;
extern detectNet::Detection *detections;
extern uchar3 *image;
extern int detection_count;
extern float input_video_width;
extern float input_video_height;

#endif //BLD_JETSON_B01

#ifdef BLD_WIN

extern cv::dnn::Net net;
extern std::vector<yolo_net::detection> yolo_detections;
extern int yolo_detection_count;
extern cv::Mat image;
extern float input_video_width;
extern float input_video_height;

#endif // BLD_WIN

extern bool valid_image_rcvd;
extern bool target_valid;
extern int target_detection_ID;
extern int target_track_ID;
extern float target_height;
extern float target_width;
extern float target_aspect;
extern float target_left;
extern float target_right;
extern float target_top;
extern float target_bottom;

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern bool target_valid;
extern int target_detection_ID;
extern int target_track_ID;
extern float target_cntr_offset_x;
extern float target_cntr_offset_y;
extern float target_height;
extern float target_width;
extern float target_aspect;
extern float target_left;
extern float target_right;
extern float target_top;
extern float target_bottom;

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
