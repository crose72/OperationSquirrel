#pragma once

#ifdef ENABLE_CV

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
#include "detect_target_yolo.h"
#include "detect_target_jetson_inference.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/
#ifdef BLD_JETSON_B01

extern videoSource* input;
extern uchar3* image;

#elif BLD_WIN

extern cv::Mat image;
extern cv::dnn::Net net;
extern std::vector<yolo_net::detection> yolo_detections;
extern int yolo_detection_count;

#else

#error "Please define a build platform."

#endif


/********************************************************************************
 * Exported objects
 ********************************************************************************/
#ifdef BLD_JETSON_B01

extern detectNet *net;
extern detectNet::Detection *detections;
extern int detection_count;


#elif BLD_WIN

extern std::vector<yolo_net::detection> yolo_detections;
extern int yolo_detection_count;

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
