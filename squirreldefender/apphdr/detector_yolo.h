#ifdef ENABLE_CV
#if defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WIN) || defined(BLD_WSL)

/********************************************************************************
 * @file    detector_yolo.h
 * @author  Cameron Rose
 * @date    1/22/2025
 ********************************************************************************/
#ifndef DETECT_TARGET_YOLO_H
#define DETECT_TARGET_YOLO_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "YOLOv8.h"
#include <vector>

/********************************************************************************
 * Exported objects
 ********************************************************************************/
#if defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WSL)

extern std::vector<Object> g_det_yolo_list;
extern int g_det_count;
;

#elif defined(BLD_WIN)

extern std::vector<YoloNet::detection> g_det_yolo_list;
extern int g_det_count;
;

#else

#error "Please define a build platform."

#endif

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/

class DetectorYOLO
{
public:
    DetectorYOLO(void);
    ~DetectorYOLO(void);

    static bool init(void);
    static void loop(void);
    static void shutdown(void);

private:
};

#endif // DETECT_TARGET_YOLO_H

#endif // defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WIN)
#endif // ENABLE_CV
