#ifdef ENABLE_CV

/********************************************************************************
 * @file    target_detection.h
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   High-level target detection arbitration that selects and manages the
 *          appropriate detection backend (YOLO or SSD). Provides a unified
 *          interface for initialization, inference, and shutdown.
 ********************************************************************************/
#ifndef DETECT_TARGET_H
#define DETECT_TARGET_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "detector_yolo.h"
#include "detector_ssd.h"

/********************************************************************************
 * Exported objects
 ********************************************************************************/

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/

class TargetDetection
{
public:
    TargetDetection(void);
    ~TargetDetection(void);

    static bool init(void);
    static void loop(void);
    static void shutdown(void);

private:
};

#endif // DETECT_TARGET_H

#endif // ENABLE_CV
