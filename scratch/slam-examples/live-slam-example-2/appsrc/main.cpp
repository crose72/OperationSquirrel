/**
 * main.cpp
 *
 * This file initializes the video input, processes video frames, and calls the SLAM functions.
 */

#include <iostream>
#include "slam.h"
#include "video_IO.h"

using namespace std;

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        cerr << "Usage: ./orb_slam3_jetson path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    // Initialize SLAM system
    if (!SlamSystem::Initialize(argv[1], argv[2]))
    {
        cerr << "Failed to initialize SLAM system" << endl;
        return -1;
    }

    // Initialize Video
    if (!Video::video_init())
    {
        cerr << "Error: Failed to open camera" << endl;
        return -1;
    }

    const double fps = 30.0;
    double timestamp = 0.0;

    // Main processing loop
    while (true)
    {
        // Process video frame
        Video::video_proc_loop();

        // Process SLAM frame
        SlamSystem::ProcessFrame(timestamp);

        // Update timestamp for the next frame
        timestamp += 1.0 / fps; // Assuming a frame rate of 30 FPS
    }

    // Shutdown SLAM system
    SlamSystem::ShutdownLoop();

    // Clean up Video
    Video::shutdown();

    return 0;
}
