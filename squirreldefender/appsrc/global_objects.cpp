/********************************************************************************
 * @file    global_objects.cpp
 * @author  Cameron Rose
 * @date    7/30/2025
 * @brief   Global variable definitions.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "global_objects.h"

/********************************************************************************
 * Object definitions
 ********************************************************************************/
bool g_app_stop = false;                 // Signals the main application to stop
bool g_app_use_video_playback = false;   // Enables video playback instead of live camera
std::string g_app_video_input_path = ""; // Path to input video for playback mode
