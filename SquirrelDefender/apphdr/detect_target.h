#pragma once

#ifdef ENABLE_CV

/********************************************************************************
 * @file    detect_target.h
 * @author  Cameron Rose
 * @date    1/22/2025
 ********************************************************************************/
#ifndef DETECT_TARGET_H
#define DETECT_TARGET_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "detect_target_yolo.h"
#include "detect_target_nv.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/
#ifdef BLD_JETSON_B01

extern videoSource* g_input;
extern uchar3* g_image;

#elif BLD_WIN

extern cv::Mat g_image;
extern cv::dnn::Net g_net;
extern std::vector<YoloNet::detection> g_yolo_detections;
extern int g_yolo_detection_count;

#else

#error "Please define a build platform."

#endif


/********************************************************************************
 * Exported objects
 ********************************************************************************/
#ifdef BLD_JETSON_B01

extern detectNet *g_net;
extern detectNet::Detection *g_detections;
extern int g_detection_count;


#elif BLD_WIN

extern std::vector<YoloNet::detection> g_yolo_detections;
extern int g_yolo_detection_count;

#else

#error "Please define a build platform."

#endif

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/

class Detection
{
public:
    Detection(void);
    ~Detection(void);

    static bool init(void);
    static void loop(void);
    static void shutdown(void);

private:
};

#endif // DETECT_TARGET_H

#endif // ENABLE_CV
