/**
 * slam.cpp
 *
 * This file implements the SLAM system initialization, frame processing, and shutdown.
 */

#include "slam.h"
#include "video_IO.h"

using namespace std;

// Define static member variables
ORB_SLAM3::System *SlamSystem::mpSLAM = nullptr;
float SlamSystem::imageScale = 1.0;
int SlamSystem::width = 1280;
int SlamSystem::height = 720;

std::vector<ORB_SLAM3::MapPoint *> SlamSystem::GetMapPoints()
{
    // Get the list of map points
    if (mpSLAM)
        return mpSLAM->GetTrackedMapPoints();
    else
        return std::vector<ORB_SLAM3::MapPoint *>();
}

std::vector<cv::KeyPoint> SlamSystem::GetKeyPoints()
{
    // Get the list of keyframes
    if (mpSLAM)
        return mpSLAM->GetTrackedKeyPointsUn();
    else
        return std::vector<cv::KeyPoint>();
}

bool SlamSystem::HasMapChanged()
{
    if (mpSLAM)
        return mpSLAM->MapChanged();
    else
        return false;
}

int SlamSystem::TrackingState()
{
    if (mpSLAM)
        return mpSLAM->GetTrackingState();
    else
        return -1; // Return an invalid state if SLAM is not initialized
}

bool SlamSystem::Initialize(const std::string &vocabPath, const std::string &settingsPath)
{
    mpSLAM = new ORB_SLAM3::System(vocabPath, settingsPath, ORB_SLAM3::System::MONOCULAR, true);
    imageScale = mpSLAM->GetImageScale();
    return (mpSLAM != nullptr);
}

void SlamSystem::ProcessFrame(double timestamp)
{
    if (valid_image_rcvd)
    {
        cv::Mat frame(height, width, CV_8UC3, image); // Create a Mat object with the data from the global `image`

        if (imageScale != 1.f)
        {
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
#endif

            width = frame.cols * imageScale;
            height = frame.rows * imageScale;
            cv::resize(frame, frame, cv::Size(width, height));

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
            double t_resize = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t_End_Resize - t_Start_Resize).count();
            mpSLAM->InsertResizeTime(t_resize);
#endif
        }

        std::chrono::system_clock::time_point t1 = std::chrono::system_clock::now();

        // Pass the image to the SLAM system
        mpSLAM->TrackMonocular(frame, timestamp); // Track the current frame

        std::chrono::system_clock::time_point t2 = std::chrono::system_clock::now();

#ifdef REGISTER_TIMES
        double t_track = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t2 - t1).count();
        mpSLAM->InsertTrackTime(t_track);
#endif
    }
}

void SlamSystem::ShutdownLoop()
{
    if (mpSLAM)
    {
        mpSLAM->Shutdown();
        delete mpSLAM;
        mpSLAM = nullptr;
    }
}
