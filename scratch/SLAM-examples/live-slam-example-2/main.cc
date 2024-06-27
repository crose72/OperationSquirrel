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

#include <iostream>
#include <opencv2/opencv.hpp>
#include "System.h"

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        std::cerr << "Usage: ./mono_tum path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

    // Open the camera
    cv::VideoCapture cap(0); // 0 is the id of the CSI camera, change if necessary
    if (!cap.isOpened())
    {
        std::cerr << "Error: Could not open the camera." << std::endl;
        return 1;
    }

    cv::Mat frame;
    while (true)
    {
        cap >> frame;
        if (frame.empty())
        {
            std::cerr << "Error: Could not capture frame." << std::endl;
            break;
        }

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(frame, cv::getTickCount() / cv::getTickFrequency());

        // Display the frame
        cv::imshow("Frame", frame);
        if (cv::waitKey(1) == 27)
        { // Press 'Esc' to exit
            break;
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    return 0;
}
