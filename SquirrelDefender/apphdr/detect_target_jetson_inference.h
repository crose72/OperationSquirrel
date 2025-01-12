#pragma once

#ifdef ENABLE_CV
#ifdef BLD_JETSON_B01

/********************************************************************************
 * @file    detect_target_jetson_inference.h
 * @author  Cameron Rose
 * @date    6/7/2023
 ********************************************************************************/
#ifndef DETECT_TARGET_SSD_H
#define DETECT_TARGET_SSD_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "jetson-utils/videoSource.h"
#include "jetson-utils/videoOutput.h"
#include "jetson-inference/detectNet.h"
#include "jetson-inference/objectTracker.h"
#include <jetson-inference/objectTrackerIOU.h>
#include <jetson-inference/objectTrackerKLT.h>
#include "video_io.h"
#include "parameters.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern videoSource *input;
extern uchar3 *image;

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern detectNet *net;
extern detectNet::Detection *detections;
extern int detection_count;

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class SSD
{
public:
    SSD(void);
    ~SSD(void);

    
    static bool init(void);
    static void loop(void);
    static void shutdown(void);

private:
};

#endif // DETECT_TARGET_SSD_H

#endif // BLD_JETSON_B01
#endif // ENABLE_CV
