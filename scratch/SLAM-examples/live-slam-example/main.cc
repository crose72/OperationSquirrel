/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <ORB_SLAM3/System.h>

using namespace std;

int main(int argc, char **argv)
{

    if (argc != 3)
    {
        cerr << "Usage: ./orb_slam3_jetson path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    // Create SLAM system
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

    float imageScale = SLAM.GetImageScale();

    // GStreamer pipeline
    std::string pipeline = "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)640, height=(int)480, format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv flip-method=2 ! video/x-raw, width=(int)640, height=(int)480, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";

    // Open camera
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

    if (!cap.isOpened())
    {
        cerr << "Error: Failed to open camera" << endl;
        return -1;
    }

    double timestamp = 0.0;
    double t_resize = 0.f;
    double t_track = 0.f;
    const double fps = 120.0;
    cv::Mat frame;

    while (true)
    {
        // Capture a frame
        cap.read(frame);

        // Check if the frame is empty
        if (frame.empty())
        {
            std::cerr << "Failed to capture frame" << std::endl;
            break;
        }

        if (imageScale != 1.f)
        {
#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
#else
            std::chrono::system_clock::time_point t_Start_Resize = std::chrono::system_clock::now();
#endif
#endif
            int width = frame.cols * imageScale;
            int height = frame.rows * imageScale;
            cv::resize(frame, frame, cv::Size(width, height));
#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
#else
            std::chrono::system_clock::time_point t_End_Resize = std::chrono::system_clock::now();
#endif
            t_resize = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t_End_Resize - t_Start_Resize).count();
            SLAM.InsertResizeTime(t_resize);
#endif
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::system_clock::time_point t1 = std::chrono::system_clock::now();
#endif

        // Pass the image to the SLAM system
        // cout << "timestamp = " << timestamp << endl;
        SLAM.TrackMonocular(frame, timestamp); // TODO change to monocular_inertial

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::system_clock::time_point t2 = std::chrono::system_clock::now();
#endif

#ifdef REGISTER_TIMES
        t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t2 - t1).count();
        SLAM.InsertTrackTime(t_track);
#endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

        // Update timestamp for the next frame
        timestamp += 1.0 / 120.0; // Assuming a frame rate of 120 FPS

        // Calculate sleep time to maintain desired frame rate
        double sleep_time = 1.0 / fps - ttrack;
        if (sleep_time > 0)
        {
            // Sleep to maintain frame rate
            std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
        }
    }

    // Release the camera
    cap.release();
    // cv::destroyAllWindows();

    // Stop SLAM system
    SLAM.Shutdown();

    return 0;
}
