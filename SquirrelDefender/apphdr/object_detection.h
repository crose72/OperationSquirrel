#pragma once

#ifdef USE_JETSON

/********************************************************************************
 * @file    object_detection.h
 * @author  Cameron Rose
 * @date    12/27/2023
 ********************************************************************************/
#ifndef OBJECT_DETECTION_H
#define OBJECT_DETECTION_H

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
#include "videoIO.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern detectNet* net;
extern detectNet::Detection* detections;
extern videoSource* input;
extern uchar3* image;
extern int numDetections;
extern uint32_t input_video_width;
extern uint32_t input_video_height;

/********************************************************************************
 * Function prototypes
 ********************************************************************************/
class Detection
{
	public:
		Detection();
		~Detection();
		
		static void initialize_detection_network(void);
		static void detection_loop(void);
		static void shutdown(void);
		static int create_detection_network(void);
		static void detect_objects(void);
		static void get_object_info(void);
		static void print_object_info(void);
		static int print_usage(void);
		static void print_performance_stats(void);
		static void delete_tracking_net(void);
	
	private:

};

class Target
{
	public:
		Target();
		~Target();

		int num_targets;
		int target_id;
	
	private:

};



#endif // OBJECT_DETECTION_H

#endif // USE_JETSON