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
#include "serial_port_handler.h"
#include "mavlink_msg_handler.h"
#include "mavlink_cmd_handler.h"
#include "scheduler.h"
#include "datalog.h"
#include "time_calc.h"


#include "videoIO.h"
#include "target_tracking.h"
#include <chrono>
#include <thread>


// Test flights
// #include "sim_flight_test_1_GPSWaypoints.h"
// #include "sim_flight_test_2_AttitudeControl.h"
// #include "sim_flight_test_3_VelocityControl.h"
#include "sim_flight_test_4_VelocityControl.h"

commandLine cmdLine(0, nullptr);
int argc;
char** argv;

bool signal_recieved = false;

// Define time interval (25 ms)
constexpr int interval_ms = 25;
//auto start_time = 0;

void sig_handler(int signo)
{
	if( signo == SIGINT )
	{
		LogVerbose("received SIGINT\n");
		signal_recieved = true;
	}
}
float err_x = 0.0;
float err_y = 0.0;
float err_x_prv = 0.0;
float err_y_prv = 0.0;
float err_x_sum = 0.0;
float err_y_sum = 0.0;

float PID(float Kp, float Ki, float Kd, float desired, float actual, float desired2, float actual2, float w1, float w2, int dim)
{
	float dt = 0.025;
	float error = (desired - actual) * w1 + (desired2 - actual2) * w2;
	float err_sum = 0.0;
	float err_prv = 0.0;
	
	if (dim == 0)
	{
		err_sum = err_x_sum;
		err_prv = err_x_prv;
	}
	else if (dim == 1)
	{
		err_sum = err_y_sum;	
		err_prv = err_y_prv;
	}
	
	float integral = (err_sum+error*dt)*Ki;
	float derivative = (error-err_prv)/dt*Kd;
	float control = error*Kp+integral*Ki+derivative*Kd;
	
	if (dim == 0)
	{
		err_x_sum = err_x_sum + error;
		err_x_prv = error;
	}
	else if (dim == 1)
	{
		err_y_sum = err_y_sum + error;
		err_y_prv = error;	
	}
	
	return control;
}

/*  Basic follow/turning
float target_velocity[3] = {0.0,0.0,0.0};
if( numDetections > 0 )
{
	for( int n=0; n < numDetections; n++ )
	{
		if (detections[n].ClassID == 1 && detections[n].Confidence > 0.8)
		{
			if (detections[n].Width() < 200 || detections[n].Height() < 700)
			{
				if (detections[n].Right <= 600)
				{
			        target_velocity[0] = 2.0;
					target_velocity[1] = -0.1;
					target_velocity[2] = 0.0;
					cmd_velocity(target_velocity);	
				}
				else
				{
			        target_velocity[0] = 2.0;
					target_velocity[1] = 0.1;
					target_velocity[2] = 0.0;
					cmd_velocity(target_velocity);	
				}

			}
			else
			{
		        target_velocity[0] = 0.0;
				target_velocity[1] = 0.0;
				target_velocity[2] = 0.0;
				cmd_velocity(target_velocity);	
			}
		}
	}
}	
*/

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


int main(void)
{
    //initialize();
    //startTask_25ms();

    // Attach signal handler to exit program
    if (signal(SIGINT, sig_handler) == SIG_ERR)
    {
        LogError("can't catch SIGINT\n");
    }

    calcStartTimeMS();
    setupTask_25ms();
    open_serial_port();
    set_message_rates();
    request_messages();
    command_line_inputs();
    input_video(cmdLine, ARG_POSITION(0));
    output_video(cmdLine, ARG_POSITION(1));
    create_detection_network();
    set_mode_GUIDED();
    arm_vehicle();
    takeoff_GPS_long((float)2.0);

    // Get the current time
    auto start_time = std::chrono::steady_clock::now();
    float desired_width;
    

    while (!signal_recieved) 
	{
        std::lock_guard<std::mutex> lock(mutex);
		calcElapsedTime();
		
		if (!capture_image())
		{
		    //break;
		}

		detect_objects();
		get_object_info();
		print_object_info();

		if (!render_output())
		{
		    //break;
		}
		
		float target_velocity[3] = {0.0,0.0,0.0};
		if( numDetections > 0 )
		{
			for( int n=0; n < numDetections; n++ )
			{
				if( detections[n].TrackID >= 0 ) // is this a tracked object?
				{			
					if (detections[n].ClassID == 1 && detections[n].Confidence > 0.8)
					{
						float Kp = 0.1;
						float Ki = 0.0;
						float Kd = 0.0;
						float x_desired = 360.0;
						float x_actual = detections[n].Height()/2.0 + detections[n].Top;
						float height_desired = 650;
						float height_actual = detections[n].Height();
						float w1 = 0.5;
						float w2 = 0.5;
						float vx_adjust = PID(Kp, Ki, Kd, x_desired, x_actual, 											height_desired, height_actual, w1, w2, 0);
						std::cout << "Control signal x: " << vx_adjust << std::endl;
						if (vx_adjust < 0)
						{
							vx_adjust = -vx_adjust;
						}
						if (height_actual >= height_desired)
						{
							vx_adjust = 0.0;
						}
						target_velocity[0] = vx_adjust;

						Kp = 0.0001;
						Ki = 0.0;
						Kd = 0.0;
						float y_desired = 640.0;
						float y_actual = detections[n].Width()/2.0 + detections[n].Left;
						float vy_adjust = -PID(Kp, Ki, Kd, y_desired, y_actual, 										0.0, 0.0, 1.0, 0.0, 1);
						std::cout << "Control signal y: " << vy_adjust << std::endl;
						target_velocity[1] = vy_adjust;

						cmd_velocity(target_velocity);	
					}
				}
				
			}
		}	

		//print_performance_stats();
		parse_serial_data();
		//test_flight();
		// logData();

		
		// Calculate elapsed time since the start of the loop
		auto end_time = std::chrono::steady_clock::now();
		auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
		

		if (firstLoopAfterStartup == true)
		{
		    firstLoopAfterStartup = false;
		}

		if (elapsed_time < interval_ms)
		{
		    auto sleep_duration = std::chrono::milliseconds(interval_ms - elapsed_time);
		    std::this_thread::sleep_for(sleep_duration);
		}

		// Update start time for the next iteration
		start_time = std::chrono::steady_clock::now();
    }

    LogVerbose("detectnet:  shutting down...\n");
    
    delete_input();
    delete_output();
    delete_tracking_net();
    
    LogVerbose("detectnet:  shutdown complete.\n");

    return 0;
}

void initialize(void)
{

}

void task_25ms(int sig, siginfo_t* si, void* uc)
{
    

}


