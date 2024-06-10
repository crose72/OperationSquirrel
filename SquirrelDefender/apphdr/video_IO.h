#pragma once

#ifdef USE_JETSON

/********************************************************************************
 * @file    video_IO.h
 * @author  Cameron Rose
 * @date    6/7/2023
 ********************************************************************************/
#ifndef VIDEO_IO_H
#define VIDEO_IO_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "jetson-utils/videoSource.h"
#include "jetson-utils/videoOutput.h"
#include "jetson-inference/detectNet.h"
#include "jetson-inference/objectTracker.h"
#include <jetson-inference/objectTrackerIOU.h>
#include "object_detection.h"
#include <signal.h>

/********************************************************************************
 * Imported objects
 ********************************************************************************/

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern videoSource* input;
extern videoOutput* output;
extern uchar3* image;
extern detectNet* net;
extern uint32_t input_video_width;
extern uint32_t input_video_height;

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class Video
{
    public:
        Video();
        ~Video();

        static bool initialize_video_streams(const commandLine& cmdLine, int positionArg);
        static bool create_input_video_stream(const commandLine& cmdLine, int positionArg);
        static bool create_output_video_stream(const commandLine& cmdLine, int positionArg);
        static void video_input_loop(void);
        static void video_output_loop(void);
        static void shutdown(void);
        static bool capture_image(void);
        static bool render_output(void);
        static void calc_video_res(void);
        static void delete_input_video_stream(void);
        static void delete_output_video_stream(void);

    private:


};

#endif // VIDEO_IO_H

#endif // USE_JETSON