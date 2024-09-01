/**
 * slam.h
 *
 * This header defines the interface for the SLAM system initialization, frame processing, and shutdown.
 */

#ifndef SLAM_H
#define SLAM_H

#include <iostream>
#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <ORB_SLAM3/System.h>
#include <ORB_SLAM3/System.h>
#include <ORB_SLAM3/MapPoint.h>
#include <ORB_SLAM3/KeyFrame.h>

class SlamSystem
{
public:
    static bool Initialize(const std::string &vocabPath, const std::string &settingsPath);
    static void ProcessFrame(double timestamp);
    static void ShutdownLoop();

    // New methods to extract information from SLAM
    static cv::Mat GetCurrentCameraPose();
    static std::vector<ORB_SLAM3::MapPoint *> GetMapPoints();
    static std::vector<cv::KeyPoint> GetKeyPoints();
    static int TrackingState();
    static bool HasMapChanged();

private:
    static ORB_SLAM3::System *mpSLAM;
    static float imageScale;
    static int width;
    static int height;
};

#endif // SLAM_H
