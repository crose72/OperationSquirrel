#pragma once

#ifdef ENABLE_CV
#if defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WIN)

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
extern cv::Mat g_image;

/********************************************************************************
 * Exported objects
 ********************************************************************************/
#ifdef BLD_JETSON_ORIN_NANO

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

#endif // defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WIN)
#endif // ENABLE_CV
