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


#include "camera_handler.h"
#include "target_tracking.h"
#include <chrono>
#include <thread>


commandLine cmdLine(0, nullptr);
int argc;
char** argv;

bool signal_recieved = false;

void sig_handler(int signo)
{
	if( signo == SIGINT )
	{
		LogVerbose("received SIGINT\n");
		signal_recieved = true;
	}
}

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
    command_line_inputs();

    // Attach signal handler
    if (signal(SIGINT, sig_handler) == SIG_ERR)
        LogError("can't catch SIGINT\n");

    input_video(cmdLine, ARG_POSITION(0));
    output_video(cmdLine, ARG_POSITION(1));
    create_detection_network(cmdLine);

    // Define time interval (25 ms)
    constexpr int interval_ms = 25;

    // Get the current time
    auto start_time = std::chrono::steady_clock::now();

    /*
     * Processing loop
     */
    while (!signal_recieved)
    {
        // Capture next image
        if (!capture_image())
        {
            break;
        }

        // Track target
        track_target(cmdLine);

        // Render output
        if (!render_output())
        {
            break;
        }

        // Print performance stats
        print_performance_stats();

        // Calculate elapsed time since the start of the loop
        auto end_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

        // If less than 25 ms have elapsed, sleep to meet the interval
        if (elapsed_time < interval_ms)
        {
            auto sleep_duration = std::chrono::milliseconds(interval_ms - elapsed_time);
            std::this_thread::sleep_for(sleep_duration);
        }

        // Update start time for the next iteration
        start_time = std::chrono::steady_clock::now();
    }

    /*
     * Destroy resources
     */
    LogVerbose("detectnet:  shutting down...\n");
    delete_input();
    delete_output();
    delete_tracking_net();

    LogVerbose("detectnet:  shutdown complete.\n");
    return 0;
}

