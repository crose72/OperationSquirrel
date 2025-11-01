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
 * Exported objects
 ********************************************************************************/

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
