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

ORB_SLAM3::System SLAM("../../ORBvoc.txt", "../imx219-83.yaml", ORB_SLAM3::System::MONOCULAR, true);
float imageScale;
std::string pipeline = "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)640, height=(int)480, format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv flip-method=2 ! video/x-raw, width=(int)640, height=(int)480, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
double timestamp = 0.0;
double t_resize = 0.f;
double t_track = 0.f;
double ttrack = 0.0f;
const double fps = 120.0;
const float obstacle_distance_threshold = 1.0; // Distance threshold to detect obstacles (1 meter)
cv::Mat frame;
Sophus::SE3f camera_pose;

void capture_image(void);
void resize_image(void);
void slam(void);
void process_frame_for_obstacles(void);
void sleep_timer(void);

void init(void);
void loop(void);
void shutdown(void);

void capture_image(void)
{
    // Capture a frame
    cap.read(frame);

    // Check if the frame is empty
    if (frame.empty())
    {
        std::cerr << "Failed to capture frame" << std::endl;
    }
}

void resize_image(void)
{
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
}

void slam(void)
{
    #ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
    std::chrono::system_clock::time_point t1 = std::chrono::system_clock::now();
#endif

    // Pass the image to the SLAM system
    // cout << "timestamp = " << timestamp << endl;
    camera_pose = SLAM.TrackMonocular(frame, timestamp); // TODO change to monocular_inertial

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
}

void sleep_timer(void)
{
    // Update timestamp for the next frame
    timestamp += 1.0 / fps; // Assuming a frame rate of 120 FPS

    // Calculate sleep time to maintain desired frame rate
    double sleep_time = 1.0 / fps - ttrack;
    if (sleep_time > 0)
    {
        // Sleep to maintain frame rate
        std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
    }
}

void process_frame_for_obstacles(void)
{
    // Extract camera position (x, y, z) from the pose matrix
    Eigen::Vector3f translation = camera_pose.translation();
    float cam_x = translation.x();
    float cam_y = translation.y();
    float cam_z = translation.z();

    std::cout << std::fixed << std::setprecision(8); // Set the precision to 2 decimal places
    std::cout << "Camera x=" << cam_x << ", y=" << cam_y << ", z=" << cam_z << std::endl;

    // Retrieve map points (obstacle candidates) from SLAM
    vector<ORB_SLAM3::MapPoint*> map_points = SLAM.GetTrackedMapPoints();
    vector<cv::Point3f> obstacles; // To store detected obstacles

    for (const auto &map_point : map_points) 
    {
        if (map_point == nullptr) 
        {
        continue; // Skip any null pointers
        }

        // Get the 3D position of each map point
        Eigen::Vector3f pos = map_point->GetWorldPos();
        float point_x = pos.x();
        float point_y = pos.y();
        float point_z = pos.z();

        // Calculate distance from camera to the map point
        float distance = std::sqrt(std::pow(point_x - cam_x, 2) + std::pow(point_y - cam_y, 2) + std::pow(point_z - cam_z, 2));

        // Check if point is within the obstacle threshold
        if (distance < obstacle_distance_threshold) {
            obstacles.push_back(cv::Point3f(point_x, point_y, point_z));
            std::cout << "Obstacle x=" << point_x << ", y=" << point_y << ", z=" << point_z
                      << " d=" << distance << " m" << std::endl;
        }
    }

    // Now you have `obstacles` containing points within the specified distance
    // Use these points for your obstacle avoidance logic
}

void init(void)
{
        imageScale = SLAM.GetImageScale();

        if (!cap.isOpened())
        {
            cerr << "Error: Failed to open camera" << endl;
        }
}

void loop(void)
{
    capture_image();
    resize_image();
    slam();
    process_frame_for_obstacles();
    sleep_timer();
}

void shutdown(void)
{
    cap.release();
    SLAM.Shutdown();
}
int main(int argc, char **argv)
{
    init();

    while (true)
    {
        loop();
    }

    shutdown();

    return 0;
}
