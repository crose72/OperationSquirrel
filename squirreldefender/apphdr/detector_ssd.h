#ifdef ENABLE_CV
#ifdef BLD_JETSON_B01

/********************************************************************************
 * @file    detector_ssd.h
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   SSD-based object detection using NVIDIA Jetson Inference on the
 *          Jetson Nano B01 (CSI camera). This module manages initialization,
 *          inference, and cleanup of the detectNet SSD model.
 ********************************************************************************/
#ifndef DETECT_TARGET_NV_H
#define DETECT_TARGET_NV_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "jetson-inference/detectNet.h"

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern detectNet *g_det_nv_net;
extern detectNet::Detection *g_det_nv_list;
extern int g_det_nv_count;

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/

class DetectorSSD
{
public:
    DetectorSSD();
    ~DetectorSSD();

    static bool init(void);
    static void loop(void);
    static void shutdown(void);

private:
};

#endif // DETECT_TARGET_NV_H

#endif // BLD_JETSON_B01
#endif // ENABLE_CV
