#ifdef ENABLE_CV

/********************************************************************************
 * @file    detect_target.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   All methods needed to initialize and create a detection network and
            choose a target.
********************************************************************************/

/********************************************************************************
* Includes
********************************************************************************/
#include "detect_target.h"

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
 * Function: Detection
 * Description: Class constructor
 ********************************************************************************/
Detection::Detection(void) {};

/********************************************************************************
 * Function: ~Detection
 * Description: Class destructor
 ********************************************************************************/
Detection::~Detection(void) {};

/********************************************************************************
 * Function: initialize_detection_net
 * Description: Delete detection network to free up resources.
 ********************************************************************************/
bool Detection::init(void)
{
#ifdef BLD_JETSON_B01

    return SSD::init();

#elif defined(BLD_JETSON_ORIN_NANO)

    return YOLO::init();

#elif defined(BLD_WIN)

    return YOLO::init();

#else

#error "Please define a build platform."

#endif
}

/********************************************************************************
 * Function: detection_loop
 * Description: Process video stream and output detected objects.
 ********************************************************************************/
void Detection::loop(void)
{
#ifdef BLD_JETSON_B01

    return SSD::loop();

#elif defined(BLD_JETSON_ORIN_NANO)

    return YOLO::loop();

#elif defined(BLD_WIN)

    return YOLO::loop();

#else

#error "Please define a build platform."

#endif
}

/********************************************************************************
 * Function: shutdown
 * Description: Shutdown detection network
 ********************************************************************************/
void Detection::shutdown(void)
{
#ifdef BLD_JETSON_B01

    return SSD::shutdown();

#elif defined(BLD_JETSON_ORIN_NANO)

    return YOLO::shutdown();

#elif defined(BLD_WIN)

    return YOLO::shutdown();

#else

#error "Please define a build platform."

#endif
}

#endif // ENABLE_CV
