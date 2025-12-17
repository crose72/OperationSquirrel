/********************************************************************************
 * @file    global_objects.cpp
 * @author  Cameron Rose
 * @date    7/30/2025
 * @brief   Global variables shared across the software system.
 ********************************************************************************/
#ifndef GLOBAL_OBJECTS_H
#define GLOBAL_OBJECTS_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include <string>

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern bool g_app_stop;
extern bool g_app_use_video_playback;
extern std::string g_app_video_input_path;

#endif // GLOBAL_OBJECTS_H
