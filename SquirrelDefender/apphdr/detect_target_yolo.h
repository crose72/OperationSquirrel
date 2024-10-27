#pragma once

#ifdef JETSON_B01

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
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <vector>
#include <string>
#include <fstream>

/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern videoSource *input;
extern uchar3 *image;
extern uint32_t input_video_width;
extern uint32_t input_video_height;
extern bool valid_image_rcvd;

/********************************************************************************
 * Exported objects
 ********************************************************************************/

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/

class YOLO
{
public:
    YOLO(void);
    ~YOLO(void);

    static void loop(void);
    static bool init(void);

    static void shutdown(void);
    static bool create_detection_network(void);
    static void detect_objects(void);
    static void delete_tracking_net(void);

private:
};

#endif // DETECT_TARGET_YOLO_H

#endif // JETSON_B01