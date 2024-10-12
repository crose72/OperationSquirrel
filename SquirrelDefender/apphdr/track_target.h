#pragma once

#ifdef JETSON_B01

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
#include "video_IO.h"
#include "detect_target.h"
#include "parameters.h"
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/cudawarping.hpp>
#include <jetson-utils/cudaMappedMemory.h> // Assuming Jetson Inference utilities are available
#include <jetson-utils/cudaRGB.h>          // For cuda functions

/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern detectNet *net;
extern detectNet::Detection *detections;
extern uchar3 *image;
extern int detection_count;
extern uint32_t input_video_width;
extern uint32_t input_video_height;
extern bool valid_image_rcvd;
extern bool target_identified;
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
extern bool target_identified;
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

    static void track_target(void);
    static void get_target_info(int n);
    static void update_target_info(void);
    static void dtrmn_target(void);
    static void tracker_init(cv::Ptr<cv::Tracker> &tracker, cv::Mat &image, cv::Rect2d &bounding_box);
    static bool tracker_update(cv::Ptr<cv::Tracker> &tracker, cv::Mat &image, cv::Rect2d &bounding_box);

private:
};

#endif // TRACK_TARGET_H

#endif // JETSON_B01