#pragma once

#include "jetson-utils/videoSource.h"
#include "jetson-utils/videoOutput.h"
#include "jetson-inference/detectNet.h"
#include "jetson-inference/objectTracker.h"
#include <signal.h>

int usage();
void cameraHandler(videoSource* input, videoOutput* output, detectNet* net, uint32_t overlayFlags);
