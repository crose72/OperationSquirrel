#pragma once

#ifdef USE_JETSON

#include "jetson-utils/videoSource.h"
#include "jetson-utils/videoOutput.h"
#include "jetson-inference/detectNet.h"
#include "jetson-inference/objectTracker.h"
#include <jetson-inference/objectTrackerIOU.h>
#include "object_detection.h"
#include <signal.h>

extern videoSource* input;
extern videoOutput* output;
extern uchar3* image;
extern detectNet* net;
extern uint32_t input_video_width;
extern uint32_t input_video_height;

class Video
{
    public:
        Video();
        ~Video();

        static void initialize_video_streams(const commandLine& cmdLine, int positionArg);
        static void video_input_loop(void);
        static void video_output_loop(void);
        static void shutdown(void);
        static int create_input_video_stream(const commandLine& cmdLine, int positionArg);
        static int create_output_video_stream(const commandLine& cmdLine, int positionArg);
        static bool capture_image(void);
        static bool render_output(void);
        static void calc_video_res(void);
        static void delete_input_video_stream(void);
        static void delete_output_video_stream(void);

    private:


};

#else
	//#error "Please define USE_JETSON to enable use of this code."
#endif