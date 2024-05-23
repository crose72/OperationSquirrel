/********************************************************************************
 * @file    target_tracking.h
 * @author  Cameron Rose
 * @date    12/27/2023
 ********************************************************************************/
#ifndef TARGET_TRACKING_H
#define TARGET_TRACKING_H

/********************************************************************************
 * Includes
 ********************************************************************************/

#include "common_inc.h"
#include "jetson-utils/videoSource.h"
#include "jetson-utils/videoOutput.h"
#include "jetson-inference/detectNet.h"
#include "jetson-inference/objectTracker.h"
#include <jetson-inference/objectTrackerIOU.h>
#include <signal.h>

extern detectNet* net;
extern detectNet::Detection* detections;
extern videoSource* input;
extern uchar3* image;
extern int numDetections;

int create_detection_network(void);
void detect_objects(void);
void get_object_info(void);
void print_object_info(void);
void print_performance_stats(void);
void delete_tracking_net(void);


#endif // TARGET_TRACKING_H
