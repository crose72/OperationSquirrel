#ifdef ENABLE_CV

/********************************************************************************
 * @file    detect_target.cpp
 * @author  Cameron Rose
 * @date    6/7/2023
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
#ifdef JETSON_B01

    return SSD::init();

#elif _WIN32

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
#ifdef JETSON_B01

    return SSD::loop();

#elif _WIN32

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
#ifdef JETSON_B01

    return SSD::shutdown();

#elif _WIN32

    return YOLO::shutdown();

#else

#error "Please define a build platform."

#endif
}

#endif // ENABLE_CV
