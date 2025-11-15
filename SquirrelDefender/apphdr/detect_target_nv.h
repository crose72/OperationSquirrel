#ifdef ENABLE_CV
#ifdef BLD_JETSON_B01

/********************************************************************************
 * @file    detect_target_nv.h
 * @author  Cameron Rose
 * @date    1/22/2025
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

#endif // DETECT_TARGET_NV_H

#endif // BLD_JETSON_B01
#endif // ENABLE_CV
