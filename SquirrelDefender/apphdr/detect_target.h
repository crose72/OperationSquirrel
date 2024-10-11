#pragma once

#ifdef JETSON_B01

/********************************************************************************
 * @file    detect_target.h
 * @author  Cameron Rose
 * @date    6/7/2023
 ********************************************************************************/
#ifndef DETECT_TARGET_H
#define DETECT_TARGET_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "jetson-utils/videoSource.h"
#include "jetson-utils/videoOutput.h"
#include "jetson-inference/detectNet.h"
#include "jetson-inference/objectTracker.h"
#include <jetson-inference/objectTrackerIOU.h>
#include "video_IO.h"
#include "parameters.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern videoSource *input;
extern uchar3 *image;
extern uint32_t input_video_width;
extern uint32_t input_video_height;

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern detectNet *net;
extern detectNet::Detection *detections;
extern int detection_count;

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class Detection
{
public:
    Detection(void);
    ~Detection(void);

    static void loop(void);
    static bool init(void);
    static void shutdown(void);
    static bool create_detection_network(void);
    static void detect_objects(void);
    static void get_object_info(void);
    static void print_object_info(void);
    static int print_usage(void);
    static void print_performance_stats(void);
    static void delete_tracking_net(void);

private:
};

#endif // DETECT_TARGET_H

#endif // JETSON_B01