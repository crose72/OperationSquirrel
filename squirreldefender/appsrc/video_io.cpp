#ifdef ENABLE_CV

/********************************************************************************
 * @file    video_io.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Configure and start video streams.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "video_io.h"

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
 * Function: Video
 * Description: Constructor of the Video class.
 ********************************************************************************/
Video::Video(void) {}

/********************************************************************************
 * Function: Video
 * Description: Constructor of the Video class.
 ********************************************************************************/
Video::~Video(void) {}

/********************************************************************************
 * Function: init
 * Description: Code to initialize video streams to run onces at the start of the
 *              program.
 ********************************************************************************/
bool Video::init(void)
{
#ifdef BLD_JETSON_B01

    return VideoNV::init();

#elif defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WSL)

    return VideoCV::init();

#elif defined(BLD_WIN)

    return VideoWin::init();

#else

#error "Please define a build platform."

#endif // BLD_JETSON_B01
}

/********************************************************************************
 * Function: in_loop
 * Description: Main video processing loop.
 ********************************************************************************/
void Video::in_loop(void)
{
#ifdef BLD_JETSON_B01

    VideoNV::in_loop();

#elif defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WSL)

    VideoCV::in_loop();

#elif defined(BLD_WIN)

    VideoWin::in_loop();

#else

#error "Please define a build platform."

#endif // BLD_JETSON_B01
}

/********************************************************************************
 * Function: out_loop
 * Description: Code needed to run each loop to  provide the video output.
 ********************************************************************************/
void Video::out_loop(void)
{
#ifdef BLD_JETSON_B01

    VideoNV::out_loop();

#elif defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WSL)

    VideoCV::out_loop();

#elif defined(BLD_WIN)

    VideoWin::out_loop();

#else

#error "Please define a build platform."

#endif // BLD_JETSON_B01
}

/********************************************************************************
 * Function: shutdown
 * Description: Shutdown detection network
 ********************************************************************************/
void Video::shutdown(void)
{
#ifdef BLD_JETSON_B01

    VideoNV::shutdown();

#elif defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WSL)

    VideoCV::shutdown();

#elif defined(BLD_WIN)

    VideoWin::shutdown();

#else

#error "Please define a build platform."

#endif // BLD_JETSON_B01
}

#endif // ENABLE_CV
