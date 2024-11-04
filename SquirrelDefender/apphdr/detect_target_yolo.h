#pragma once

#ifdef ENABLE_CV
#ifdef _WIN32

/********************************************************************************
 * @file    detect_target_yolo.h
 * @author  Cameron Rose
 * @date    6/7/2023
 ********************************************************************************/
#ifndef DETECT_TARGET_YOLO_H
#define DETECT_TARGET_YOLO_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "video_IO.h"
#include "parameters.h"
#include "yolo_net.h"
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <vector>
#include <string>
#include <fstream>
#include <string>

/********************************************************************************
 * Imported objects
 ********************************************************************************/
#ifdef JETSON_B01

extern uchar3* image;

#elif _WIN32

extern cv::Mat image;

#else

#error "Please define a build platform."

#endif


/********************************************************************************
 * Exported objects
 ********************************************************************************/
#ifdef JETSON_B01

/* None */

#elif _WIN32

extern std::vector<yolo_net::detection> yolo_detections;
extern int yolo_detection_count;

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

#endif // _WIN32
#endif // ENABLE_CV
