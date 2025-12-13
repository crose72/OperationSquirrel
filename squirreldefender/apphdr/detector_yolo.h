#ifdef ENABLE_CV
#if defined(BLD_JETSON_ORIN) || defined(BLD_WIN) || defined(BLD_WSL)

/********************************************************************************
 * @file    detector_yolo.h
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   YOLO-based object detection using TensorRT or ONNXRuntime depending
 *          on platform. This module manages initialization, inference, and
 *          shutdown of the YOLO engine through a static namespace-style class.
 ********************************************************************************/
#ifndef DETECT_TARGET_YOLO_H
#define DETECT_TARGET_YOLO_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include <vector>

#include "YOLOv8.h"

/********************************************************************************
 * Exported objects
 ********************************************************************************/
#if defined(BLD_JETSON_ORIN) || defined(BLD_WSL)

// Native linux supports TensorRT based inference
extern std::vector<Object> g_det_yolo_list;
extern int g_det_count;

#elif defined(BLD_WIN)

// Windows uses ONNXRuntime based inference
extern std::vector<YoloNet::detection> g_det_yolo_list;
extern int g_det_count;

#else

#error "Please define a build platform."

#endif

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/

class DetectorYOLO
{
public:
    DetectorYOLO();
    ~DetectorYOLO();

    static bool init(void);
    static void loop(void);
    static void shutdown(void);

private:
};

#endif // DETECT_TARGET_YOLO_H

#endif // defined(BLD_JETSON_ORIN) || defined(BLD_WIN) || defined(BLD_WSL)
#endif // ENABLE_CV
