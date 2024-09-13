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

#include "jetson-utils/videoSource.h"
#include "jetson-utils/videoOutput.h"
#include "jetson-inference/detectNet.h"
#include "jetson-inference/objectTracker.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <signal.h>
#include <string>
#include <sstream>

bool countdown_active = false;
bool countdown_started = false; // Track if the countdown has started
bool signal_received = false;
bool image_captured = false; // This prevents multiple images from being captured after the countdown
int sigint_count = 0;
int image_counter = 0; // Global counter for image filenames
const int required_sigint_count = 3;
std::chrono::steady_clock::time_point last_sigint_time;
std::chrono::seconds sigint_time_window(5); // Calibratable time window
std::mutex image_mutex;
int capture_delay = 10;

float box_width;
float box_height;
float box_center_x;
float box_center_y;
uint32_t class_id;

// Timer variable
std::chrono::steady_clock::time_point countdown_start;
std::chrono::steady_clock::time_point countdown_end;

void start_countdown()
{
    countdown_start = std::chrono::steady_clock::now();
    countdown_end = countdown_start + std::chrono::seconds(capture_delay);
    countdown_active = true;
}

std::string generate_filename()
{
    std::stringstream filename;
    filename << "captured_image_" << std::setfill('0') << std::setw(4) << image_counter << ".png";
    image_counter++; // Increment the counter for the next image
    return filename.str();
}

bool is_centered(detectNet::Detection detection, int frame_width, int frame_height)
{
    // Calculate the center of the frame
    float center_x = frame_width / 2.0;
    float center_y = frame_height / 2.0;

    // Get the bounding box center
    float bbox_center_x = (detection.Left + detection.Right) / 2.0;
    float bbox_center_y = (detection.Top + detection.Bottom) / 2.0;

    // Define some threshold for being "centered" (you can adjust these)
    float threshold_x = frame_width * 0.05; // 10% threshold
    float threshold_y = frame_height * 0.05;

    return (abs(bbox_center_x - center_x) < threshold_x && abs(bbox_center_y - center_y) < threshold_y);
}

void process_detections(detectNet::Detection *detections, int numDetections, videoOutput *output, int frame_width, int frame_height, uchar3 *image)
{
    for (int n = 0; n < numDetections; n++)
    {
        detectNet::Detection &detection = detections[n];

        // Determine if the object is centered
        if (is_centered(detection, frame_width, frame_height))
        {
            // If centered, show a green border
            cv::Mat frame(cv::Size(frame_width, frame_height), CV_8UC3, (void *)image, cv::Mat::AUTO_STEP);
            cv::rectangle(frame, cv::Point(0, 0), cv::Point(frame_width, frame_height), cv::Scalar(0, 255, 0), 4);

            if (detections[n].ClassID == 1 && detections[n].Confidence > 0.5)
            {
                // Print the size, width, and center of the bounding box
                box_width = detection.Width();
                box_height = detection.Height();
                box_center_x = (detection.Left + detection.Right) / 2.0;
                box_center_y = (detection.Top + detection.Bottom) / 2.0;
                class_id = detections[n].ClassID;

                std::cout << "Bounding box: width=" << box_width << ", height=" << box_height << std::endl;
                std::cout << "Center of box: (" << box_center_x << ", " << box_center_y << ")" << std::endl;
            }
        }
    }
}

void capture_image(uchar3 *image, int frame_width, int frame_height)
{
    // Only print for the "person" class, which has ClassID == 1

    // Convert the image to OpenCV format
    cv::Mat frame(cv::Size(frame_width, frame_height), CV_8UC3, (void *)image, cv::Mat::AUTO_STEP);

    // Prepare the text to overlay (Box width and height)
    std::stringstream ss;
    ss << "Width: " << box_width << " Height: " << box_height << "Center X: " << box_center_x << " Center Y: " << box_center_y << "Class: " << class_id;

    // Overlay the text on the image at the upper right-hand corner
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.6;
    int thickness = 1;
    int baseline = 0;
    cv::Size textSize = cv::getTextSize(ss.str(), fontFace, fontScale, thickness, &baseline);

    // Calculate position: upper right-hand corner
    cv::Point textOrg(frame_width - textSize.width - 10, textSize.height + 10); // Adjust as needed

    // Draw the text on the image
    cv::putText(frame, ss.str(), textOrg, fontFace, fontScale, cv::Scalar(255, 255, 255), thickness);

    // Generate the filename
    std::string filename = generate_filename();

    // Save the image
    cv::imwrite(filename, frame);

    std::cout << "Captured image: " << filename << std::endl;
}

void process_image_capture(uchar3 *image, int frame_width, int frame_height)
{
    if (countdown_started && !image_captured && std::chrono::steady_clock::now() >= countdown_end)
    {
        std::cout << "Countdown finished. Capturing image..." << std::endl;
        capture_image(image, frame_width, frame_height); // Capture image after countdown
        image_captured = true;                           // Mark the image as captured
        countdown_started = false;                       // Reset countdown flag
        countdown_active = false;                        // Reset active flag after image capture
    }
}

void sig_handler(int signo)
{
    if (signo == SIGINT)
    {
        auto current_time = std::chrono::steady_clock::now();

        // Check if the time since the last SIGINT is within the time window
        if (std::chrono::duration_cast<std::chrono::seconds>(current_time - last_sigint_time) < sigint_time_window)
        {
            sigint_count++;
            std::cout << "SIGINT received (" << sigint_count << "/" << required_sigint_count << ")" << std::endl;
        }
        else
        {
            // Reset if time window expired
            sigint_count = 1;
            std::cout << "First SIGINT received after time window." << std::endl;
        }

        last_sigint_time = current_time;

        // If 3 SIGINT signals have been received in the time window, shut down
        if (sigint_count >= required_sigint_count)
        {
            std::cout << "Exiting program (3 SIGINT signals received)." << std::endl;
            signal_received = true;
        }
        else if (!countdown_started) // Add this condition to start the countdown if it hasn't started yet
        {
            std::cout << "SIGINT received. Starting 5-second countdown..." << std::endl;
            start_countdown();        // Start the countdown
            countdown_started = true; // Mark countdown as started
            image_captured = false;   // Reset the image capture flag
        }
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

int main(int argc, char **argv)
{
    /*
     * parse command line
     */
    commandLine cmdLine(argc, argv);

    if (cmdLine.GetFlag("help"))
        return usage();

    /*
     * attach signal handler
     */
    if (signal(SIGINT, sig_handler) == SIG_ERR)
        LogError("can't catch SIGINT\n");

    /*
     * create input stream
     */
    videoOptions options1;

    options1.resource = URI("csi://0");
    options1.resource.protocol = "csi";
    options1.resource.location = "0";
    options1.deviceType = videoOptions::DeviceType::DEVICE_CSI;
    options1.ioType = videoOptions::IoType::INPUT;
    options1.width = 1280;
    options1.height = 720;
    options1.frameRate = 30;
    options1.numBuffers = 4;
    options1.zeroCopy = true;
    options1.flipMethod = videoOptions::FlipMethod::FLIP_NONE;

    videoSource *input = videoSource::Create(options1);

    if (!input)
    {
        LogError("detectnet:  failed to create input stream\n");
        return 1;
    }

    /*
     * create output stream
     */
    videoOptions options2;

    options2.resource = "display://0"; // Specify the display URI
    options2.resource.protocol = "display";
    options2.resource.location = "0";
    options2.deviceType = videoOptions::DeviceType::DEVICE_DISPLAY;
    options2.ioType = videoOptions::IoType::OUTPUT;
    options2.width = 1920;
    options2.height = 1080;
    options2.frameRate = 30; // Adjust as needed
    options2.numBuffers = 4;
    options2.zeroCopy = true;

    videoOutput *output = videoOutput::Create(options2);

    if (!output)
    {
        LogError("detectnet:  failed to create output stream\n");
        return 1;
    }

    /*
     * create detection network
     */
    detectNet *net = detectNet::Create(cmdLine);

    if (!net)
    {
        LogError("detectnet:  failed to load detectNet model\n");
        return 1;
    }

    // parse overlay flags
    const uint32_t overlayFlags = detectNet::OverlayFlagsFromStr(cmdLine.GetString("overlay", "box,labels,conf"));

    /*
     * processing loop
     */
    while (!signal_received)
    {
        // capture next image
        uchar3 *image = NULL;
        int status = 0;

        if (!input->Capture(&image, &status))
        {
            if (status == videoSource::TIMEOUT)
                continue;

            break; // EOS
        }

        // detect objects in the frame
        detectNet::Detection *detections = NULL;
        const int numDetections = net->Detect(image, input->GetWidth(), input->GetHeight(), &detections, overlayFlags);

        if (numDetections > 0)
        {
            // Process detections for object centering and capture logic
            process_detections(detections, numDetections, output, input->GetWidth(), input->GetHeight(), image);
        }

        // Render outputs
        if (output != NULL)
        {
            output->Render(image, input->GetWidth(), input->GetHeight());

            // update the status bar
            char str[256];
            sprintf(str, "TensorRT %i.%i.%i | %s | Network %.0f FPS", NV_TENSORRT_MAJOR, NV_TENSORRT_MINOR, NV_TENSORRT_PATCH, precisionTypeToStr(net->GetPrecision()), net->GetNetworkFPS());
            output->SetStatus(str);

            // check if the user quit
            if (!output->IsStreaming())
                break;
        }

        // Check if fewer than 3 SIGINT signals were received in the time window
        if (sigint_count == 1 && std::chrono::steady_clock::now() - last_sigint_time > sigint_time_window)
        {
            std::cout << "SIGINT not repeated 3 times within the time window. Capturing image." << std::endl;
            process_image_capture(image, 1280, 720);
            sigint_count = 0; // Reset count after image capture
        }

        // print out timing info
        net->PrintProfilerTimes();
    }

    // Cleanup
    SAFE_DELETE(input);
    SAFE_DELETE(output);
    SAFE_DELETE(net);

    std::cout << "Program exiting..." << std::endl;
    return 0;
}
