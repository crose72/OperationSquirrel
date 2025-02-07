#pragma once

#ifdef ENABLE_CV

/********************************************************************************
 * @file    detect_target_yolo.h
 * @author  Cameron Rose
 * @date    1/22/2025
 ********************************************************************************/
#ifndef DETECT_TARGET_YOLO_H
#define DETECT_TARGET_YOLO_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "video_io.h"
#include "yolo_net.h"
#include "yolov8.h"
#include <opencv2/cudaimgproc.hpp>
#include <vector>

/********************************************************************************
 * Imported objects
 ********************************************************************************/
#ifdef BLD_JETSON_B01

extern uchar3* g_image;

#elif defined(BLD_JETSON_ORIN_NANO)

extern cv::Mat g_image;

#elif defined(BLD_WIN)

extern cv::Mat g_image;

#else

#error "Please define a build platform."

#endif


/********************************************************************************
 * Exported objects
 ********************************************************************************/
#ifdef BLD_JETSON_B01

/* None */

#elif defined(BLD_JETSON_ORIN_NANO)

extern std::vector<Object> g_yolo_detections;
extern int g_yolo_detection_count;

#elif defined(BLD_WIN)

extern std::vector<YoloNet::detection> g_yolo_detections;
extern int g_yolo_detection_count;

#else

#error "Please define a build platform."

#endif

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/

class YOLO
{
public:
    YOLO(void);
    ~YOLO(void);

    static bool init(void);
    static void loop(void);
    static void shutdown(void);

private:
};

#endif // DETECT_TARGET_YOLO_H

#endif // ENABLE_CV
