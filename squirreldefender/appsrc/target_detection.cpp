#ifdef ENABLE_CV

/********************************************************************************
 * @file    target_detection.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   All methods needed to initialize and create a detection network and
            choose a target.
********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "target_detection.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: TargetDetection
 * Description: Class constructor
 ********************************************************************************/
TargetDetection::TargetDetection(void) {};

/********************************************************************************
 * Function: ~TargetDetection
 * Description: Class destructor
 ********************************************************************************/
TargetDetection::~TargetDetection(void) {};

/********************************************************************************
 * Function: initialize_detection_net
 * Description: Delete detection network to free up resources.
 ********************************************************************************/
bool TargetDetection::init(void)
{
#ifdef BLD_JETSON_B01

    return DetectorSSD::init();

#elif defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WSL)

    return DetectorYOLO::init();

#elif defined(BLD_WIN)

    return DetectorYOLO::init();

#else

#error "Please define a build platform."

#endif
}

/********************************************************************************
 * Function: detection_loop
 * Description: Process video stream and output detected objects.
 ********************************************************************************/
void TargetDetection::loop(void)
{
#ifdef BLD_JETSON_B01

    return DetectorSSD::loop();

#elif defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WSL)

    return DetectorYOLO::loop();

#elif defined(BLD_WIN)

    return DetectorYOLO::loop();

#else

#error "Please define a build platform."

#endif
}

/********************************************************************************
 * Function: shutdown
 * Description: Shutdown detection network
 ********************************************************************************/
void TargetDetection::shutdown(void)
{
#ifdef BLD_JETSON_B01

    return DetectorSSD::shutdown();

#elif defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WSL)

    return DetectorYOLO::shutdown();

#elif defined(BLD_WIN)

    return DetectorYOLO::shutdown();

#else

#error "Please define a build platform."

#endif
}

#endif // ENABLE_CV
