#pragma once

#include "jetson-utils/videoSource.h"
#include "jetson-utils/videoOutput.h"
#include "jetson-inference/detectNet.h"
#include "jetson-inference/objectTracker.h"
#include <jetson-inference/objectTrackerIOU.h>
#include "target_tracking.h"
#include <signal.h>

extern videoSource* input;
extern videoOutput* output;
extern uchar3* image;
extern detectNet* net;

class Video
{
    public:
        Video();
        ~Video();

        static int create_input_video_stream(const commandLine& cmdLine, int positionArg);
        static int create_output_video_stream(const commandLine& cmdLine, int positionArg);
        static bool capture_image(void);
        static bool render_output(void);
        static void delete_input_video_stream(void);
        static void delete_output_video_stream(void);

    private:


};




