#pragma once

#include "jetson-utils/videoSource.h"
#include "jetson-utils/videoOutput.h"
#include "jetson-inference/detectNet.h"
#include "jetson-inference/objectTracker.h"
#include <jetson-inference/objectTrackerIOU.h>
#include <signal.h>

extern videoSource* input;
extern videoOutput* output;
extern uchar3* image;
extern detectNet* net;

int input_video(const commandLine& cmdLine, int positionArg);
int output_video(const commandLine& cmdLine, int positionArg);
bool capture_image(void);
bool render_output(void);
void delete_input(void);
void delete_output(void);
