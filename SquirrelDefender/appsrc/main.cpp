/*
 * Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "common_inc.h"
#include "global_objects.h"
#include "global_calibrations.h"
#include "serial_comm.h"
#include "mavlink_msg_handler.h"
#include "mavlink_cmd_handler.h"
#include "vehicle_controller.h"
#include "follow_target.h"
#include "scheduler.h"
#include "datalog.h"
#include "time_calc.h"

#ifdef USE_JETSON
	#include "videoIO.h"
	#include "target_tracking.h"
	#include <jsoncpp/json/json.h> //sudo apt-get install libjsoncpp-dev THEN target_link_libraries(your_executable_name jsoncpp)
#endif

#ifdef USE_JETSON
int argc;
char** argv;
bool signal_recieved = false;
commandLine cmdLine(0, nullptr);
#endif

#ifdef USE_JETSON
int usage()
{
	printf("usage: detectnet [--help] [--network=NETWORK] [--threshold=THRESHOLD] ...\n");
	printf("                 input [output]\n\n");
	printf("Locate objects in a video/image stream using an object detection DNN.\n");
	printf("See below for additional arguments that may not be shown above.\n\n");
	printf("positional arguments:\n");
	printf("    input           resource URI of input stream  (see videoSource below)\n");
	printf("    output          resource URI of output stream (see videoOutput below)\n\n");

	printf("%s", detectNet::Usage());
	printf("%s", objectTracker::Usage());
	printf("%s", videoSource::Usage());
	printf("%s", videoOutput::Usage());
	printf("%s", Log::Usage());

	return 0;
}

int command_line_inputs(void)
{
	/*
	 * parse command line
	 */
	commandLine cmdLine(argc, argv);

	if( cmdLine.GetFlag("help") )
		return usage();
	
	return 1;

}
#endif

void sig_handler(int signo)
{
	if( signo == SIGINT )
	{
		#ifdef USE_JETSON
		LogVerbose("received SIGINT\n");
		#else
			//#error "Please define USE_JETSON to enable use of this code."
		#endif
		signal_recieved = true;
	}
}

void attach_sig_handler(void)
{
	// This will allow the program to stop of Ctl+C is received
    if (signal(SIGINT, sig_handler) == SIG_ERR)
    {
		#ifdef USE_JETSON
        LogError("can't catch SIGINT\n");
		#endif
		
    }
}

int main(void)
{
	TimeCalc Time;

    Time.calc_app_start_time();
	attach_sig_handler();
    MavMsg::start_mav_comm();
    MavMsg::message_subscriptions();
    
	#ifdef USE_JETSON
		command_line_inputs();
		Video::initialize_video_streams(cmdLine, ARG_POSITION(0));
		create_detection_network();
	#endif
	
    MavCmd::set_mode_GUIDED();
    MavCmd::arm_vehicle();
    MavCmd::takeoff_GPS_long((float)2.0);

    while (!signal_recieved) 
	{
        std::lock_guard<std::mutex> lock(mutex);
		Time.calc_elapsed_time();
		MavMsg::parse_mav_msgs();
		
		#ifdef USE_JETSON
		

		Follow::follow_target();
		#endif	

		//print_performance_stats();

        Time.loop_rate_controller();

		if (firstLoopAfterStartup == true)
		{
		    firstLoopAfterStartup = false;
		}
		// logData();
		Time.calc_loop_start_time();
    }

	#ifdef USE_JETSON
		LogVerbose("detectnet:  shutting down...\n");
		Video::delete_input_video_stream();
		Video::delete_output_video_stream();
		delete_tracking_net();
		LogVerbose("detectnet:  shutdown complete.\n");
	#endif

    MavMsg::stop_mav_comm();

    return 0;
}