#pragma once

#ifdef ENABLE_CV

/********************************************************************************
 * @file    detect_target_yolo.h
 * @author  Cameron Rose
 * @date    6/7/2023
 ********************************************************************************/
#ifndef YOLO_NET_H
#define YOLO_NET_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "parameters.h"
#include "video_io.h"
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <vector>
#include <string>
#include <fstream>

/********************************************************************************
 * Imported objects
 ********************************************************************************/

/********************************************************************************
 * Exported objects
 ********************************************************************************/

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/

class yolo_net
{
public:
    yolo_net(void);
    ~yolo_net(void);

    struct detection
    {
        // Detection Info
        uint32_t ClassID;    /**< Class index of the detected object. */
        float Confidence;    /**< Confidence value of the detected object. */

        // Bounding Box Coordinates
        float Left;          /**< Left bounding box coordinate (in pixels) */
        float Right;         /**< Right bounding box coordinate (in pixels) */
        float Top;           /**< Top bounding box coordinate (in pixels) */
        float Bottom;        /**< Bottom bounding box coordinate (in pixels) */

        // Utility functions
        inline float Width() const { return Right - Left; }
        inline float Height() const { return Bottom - Top; }
        inline float Area() const { return Width() * Height(); }
        inline void Center(float* x, float* y) const { if (x) *x = Left + Width() * 0.5f; if (y) *y = Top + Height() * 0.5f; }

        // Reset function
        inline void reset() { ClassID = 0; Confidence = 0; Left = 0; Right = 0; Top = 0; Bottom = 0; }

        // Default constructor
        inline detection() { reset(); }
    };

    static cv::dnn::Net create(const std::string& model, const std::string& class_list_path, int backend_id, int target_id);
    static void detect(cv::Mat& input, cv::dnn::Net net, std::vector<yolo_net::detection>& detections);

private:
    static const float INPUT_WIDTH;
    static const float INPUT_HEIGHT;
    static const float SCORE_THRESHOLD;
    static const float NMS_THRESHOLD;
    static const float CONFIDENCE_THRESHOLD;

    static const float FONT_SCALE;
    static const int FONT_FACE;
    static const int THICKNESS;

    static const cv::Scalar BLACK;
    static const cv::Scalar BLUE;
    static const cv::Scalar YELLOW;
    static const cv::Scalar RED;

    //static std::vector<std::string> class_list;

    static std::vector<cv::Mat> pre_process(cv::Mat& input, cv::dnn::Net& net);
    static void clear_prev_detections(std::vector<yolo_net::detection>& detections);
    static void unwrap_detections(float* data, int rows, float x_factor, float y_factor,
        std::vector<int>& class_ids, std::vector<float>& confidences,
        std::vector<cv::Rect>& boxes, std::vector<yolo_net::detection>& detections,
        const std::vector<std::string>& class_list);
    static void draw_bounding_boxes(cv::Mat& input, const std::vector<int>& indices,
        const std::vector<cv::Rect>& boxes, const std::vector<float>& confidences,
        const std::vector<int>& class_ids, const std::vector<std::string>& class_list,
        std::vector<yolo_net::detection>& detections);
    static void nms_suppression(const std::vector<cv::Rect>& boxes, const std::vector<float>& confidences, std::vector<int>& indices);
    static void post_process(cv::Mat& input, std::vector<cv::Mat>& outputs, std::vector<yolo_net::detection>& detections, const std::vector<std::string>& class_list);
    static void draw_label(cv::Mat& input, std::string label, int left, int top);

};

#endif // YOLO_NET_H

#endif // ENABLE_CV
