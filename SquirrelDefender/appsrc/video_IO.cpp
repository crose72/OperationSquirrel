#ifdef ENABLE_CV

/********************************************************************************
 * @file    videoIO.cpp
 * @author  Cameron Rose
 * @date    5/22/2024
 * @brief   Configure and start video streams, and output_vid_disp video for other
            software components.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "video_IO.h"

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
#ifdef JETSON_B01

    return VideoNV::init();

#elif _WIN32

    return VideoWin::init();

#else

#error "Please define a build platform."

#endif // JETSON_B01
}

/********************************************************************************
 * Function: in_loop
 * Description: Main video processing loop.
 ********************************************************************************/
void Video::in_loop(void)
{
#ifdef JETSON_B01

    VideoNV::in_loop();

#elif _WIN32

    VideoWin::in_loop();

#else

#error "Please define a build platform."

#endif // JETSON_B01
}

/********************************************************************************
 * Function: out_loop
 * Description: Code needed to run each loop to  provide the video output.
 ********************************************************************************/
void Video::out_loop(void)
{
#ifdef JETSON_B01

    VideoNV::out_loop();

#elif _WIN32

    VideoWin::out_loop();

#else

#error "Please define a build platform."

#endif // JETSON_B01
}

/********************************************************************************
 * Function: shutdown
 * Description: Shutdown detection network
 ********************************************************************************/
void Video::shutdown(void)
{
#ifdef JETSON_B01

    VideoNV::shutdown();

#elif _WIN32

    VideoWin::shutdown();

#else

#error "Please define a build platform."

#endif // JETSON_B01
}

#endif // ENABLE_CV
