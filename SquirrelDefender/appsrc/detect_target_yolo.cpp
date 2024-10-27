#ifdef JETSON_B01

/********************************************************************************
 * @file    target_tracking.cpp
 * @author  Cameron Rose
 * @date    6/7/2023
 * @brief   All methods needed to initialize and create a detection network and
            choose a target.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "detect_target_yolo.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/

/* For yolo */
cv::dnn::Net net_yolo;
std::vector<std::string> class_list_yolo;
std::vector<cv::Mat> detections_yolo;
int detection_count_yolo;

cv::Mat image_yolo_wrapped;
bool initialized_yolo_image;
float input_vid_width;
float input_vid_height;
const float INPUT_WIDTH = 640.0;
const float INPUT_HEIGHT = 640.0;
const float SCORE_THRESHOLD = 0.5;
const float NMS_THRESHOLD = 0.45;
const float CONFIDENCE_THRESHOLD = 0.45;

// Text parameters.
const float FONT_SCALE = 0.7;
const int FONT_FACE = cv::FONT_HERSHEY_SIMPLEX;
const int THICKNESS = 1;

// Colors.
cv::Scalar BLACK = cv::Scalar(0, 0, 0);
cv::Scalar BLUE = cv::Scalar(255, 178, 50);
cv::Scalar YELLOW = cv::Scalar(0, 255, 255);
cv::Scalar RED = cv::Scalar(0, 0, 255);

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: YOLO
 * Description: Class constructor
 ********************************************************************************/

/********************************************************************************
 * Function: ~YOLO
 * Description: Class destructor
 ********************************************************************************/

void draw_label(cv::Mat &input, std::string label, int left, int top)
{
    // Display the label at the top of the bounding box.
    int baseLine;
    cv::Size label_size = cv::getTextSize(label, FONT_FACE, FONT_SCALE, THICKNESS, &baseLine);
    top = std::max(top, label_size.height);
    // Top left corner.
    cv::Point tlc = cv::Point(left, top);
    // Bottom right corner.
    cv::Point brc = cv::Point(left + label_size.width, top + label_size.height + baseLine);
    // Draw white rectangle.
    cv::rectangle(input, tlc, brc, BLACK, cv::FILLED);
    // Put the label on the black rectangle.
    cv::putText(input, label, cv::Point(left, top + label_size.height), FONT_FACE, FONT_SCALE, YELLOW, THICKNESS);
}

std::vector<cv::Mat> pre_process(cv::Mat &input, cv::dnn::Net &net)
{
    // Convert to blob.
    cv::Mat blob;
    cv::dnn::blobFromImage(input, blob, 1. / 255., cv::Size(INPUT_WIDTH, INPUT_HEIGHT), cv::Scalar(), true, false);

    net.setInput(blob);

    // Forward propagate.
    std::vector<cv::Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());

    return outputs;
}

cv::Mat post_process(cv::Mat &input, std::vector<cv::Mat> &outputs, const std::vector<std::string> &class_name)
{
    // Initialize vectors to hold respective outputs while unwrapping detections.
    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    // Resizing factor.
    float x_factor = input.cols / INPUT_WIDTH;
    float y_factor = input.rows / INPUT_HEIGHT;
    float *data = (float *)outputs[0].data;
    const int dimensions = 85;
    // 25200 for default size 640.
    const int rows = 25200;
    // Iterate through 25200 detections.
    for (int i = 0; i < rows; ++i)
    {
        float confidence = data[4];
        // Discard bad detections and continue.
        if (confidence >= CONFIDENCE_THRESHOLD)
        {
            float *classes_scores = data + 5;
            // Create a 1x85 Mat and store class scores of 80 classes.
            cv::Mat scores(1, class_name.size(), CV_32FC1, classes_scores);
            // Perform minMaxLoc and acquire the index of best class score.
            cv::Point class_id;
            double max_class_score;
            cv::minMaxLoc(scores, 0, &max_class_score, 0, &class_id);
            // Continue if the class score is above the threshold.
            if (max_class_score > SCORE_THRESHOLD)
            {
                // Store class ID and confidence in the pre-defined respective vectors.
                confidences.push_back(confidence);
                class_ids.push_back(class_id.x);
                // Center.
                float cx = data[0];
                float cy = data[1];
                // Box dimension.
                float w = data[2];
                float h = data[3];
                // Bounding box coordinates.
                int left = int((cx - 0.5 * w) * x_factor);
                int top = int((cy - 0.5 * h) * y_factor);
                int width = int(w * x_factor);
                int height = int(h * y_factor);
                // Store good detections in the boxes vector.
                boxes.push_back(cv::Rect(left, top, width, height));
            }
        }
        // Jump to the next row.
        data += 85;
    }

    // Perform Non-Maximum Suppression and draw predictions.
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, indices);
    for (int i = 0; i < indices.size(); i++)
    {
        int idx = indices[i];
        cv::Rect box = boxes[idx];
        int left = box.x;
        int top = box.y;
        int width = box.width;
        int height = box.height;
        // Draw bounding box.
        cv::rectangle(input, cv::Point(left, top), cv::Point(left + width, top + height), BLUE, 3 * THICKNESS);
        // Get the label for the class name and its confidence.
        std::string label = cv::format("%.2f", confidences[idx]);
        label = class_name[class_ids[idx]] + ":" + label;
        // Draw class labels.
        draw_label(input, label, left, top);
    }
    return input;
}

/********************************************************************************
 * Function: YOLO
 * Description: Class constructor
 ********************************************************************************/
YOLO::YOLO(void) {};

/********************************************************************************
 * Function: ~YOLO
 * Description: Class destructor
 ********************************************************************************/
YOLO::~YOLO(void) {};

/********************************************************************************
 * Function: create_detection_network
 * Description: Create YOLO detection network with parameters.
 ********************************************************************************/
bool YOLO::create_detection_network(void)
{
    /* Load class list */
    std::ifstream ifs("../networks/yolov5m/coco.names");
    std::string line;

    while (std::getline(ifs, line))
    {
        class_list_yolo.push_back(line);
    }

    /* Load model and enable GPU processing */
    net_yolo = cv::dnn::readNet("../networks/yolov5m/yolov5m_float32_modified.onnx");
    net_yolo.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    net_yolo.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

    return true;
};

/********************************************************************************
 * Function: detect_objects
 * Description: Process the video feed and return detected objects.
 ********************************************************************************/
void YOLO::detect_objects(void)
{
    if (valid_image_rcvd && !initialized_yolo_image)
    {
        image_yolo_wrapped = cv::Mat(input_video_height, input_video_width, CV_8UC3, image); // Directly wrap uchar3*
        initialized_yolo_image = true;
    }
    else if (valid_image_rcvd && initialized_yolo_image)
    {
        detections_yolo = pre_process(image_yolo_wrapped, net_yolo);
        image_yolo_wrapped = post_process(image_yolo_wrapped, detections_yolo, class_list_yolo);
    }
};

/********************************************************************************
 * Function: init
 * Description: Take care of startup functions and initializations.
 ********************************************************************************/
bool YOLO::init(void)
{
    initialized_yolo_image = false;
    input_vid_width = static_cast<float>(input_video_width);
    input_vid_height = static_cast<float>(input_video_height);

    create_detection_network();

    return true;
};

/********************************************************************************
 * Function: loop
 * Description: Manage functions that run every loop.
 ********************************************************************************/
void YOLO::loop(void)
{
    detect_objects();
};

/********************************************************************************
 * Function: shutdown
 * Description: Clean up before shutting down.
 ********************************************************************************/
void YOLO::shutdown(void) {
    // nothing yet
};

#endif // JETSON_B01