
#ifdef ENABLE_CV
#ifdef BLD_WIN

/********************************************************************************
* @file    yolo_net.cpp
* @author  Cameron Rose
* @date    1/22/2025
* @brief   All methods needed to initialize and create a detection network and
        choose a target.
********************************************************************************/

/********************************************************************************
* Includes
********************************************************************************/
#include "yolo_net.h"

/********************************************************************************
* Typedefs
********************************************************************************/

/********************************************************************************
* Private macros and defines
********************************************************************************/

/********************************************************************************
* Object definitions
********************************************************************************/
std::vector<cv::Mat> outputs;
std::vector<std::string> class_list;

const float YoloNet::INPUT_WIDTH = 640.0f;
const float YoloNet::INPUT_HEIGHT = 640.0f;
const float YoloNet::SCORE_THRESHOLD = 0.5f;
const float YoloNet::NMS_THRESHOLD = 0.45f;
const float YoloNet::CONFIDENCE_THRESHOLD = 0.45f;

const float YoloNet::FONT_SCALE = 0.7f;
const int YoloNet::FONT_FACE = cv::FONT_HERSHEY_SIMPLEX;
const int YoloNet::THICKNESS = 1;

const cv::Scalar YoloNet::BLACK = cv::Scalar(0, 0, 0);
const cv::Scalar YoloNet::BLUE = cv::Scalar(255, 178, 50);
const cv::Scalar YoloNet::YELLOW = cv::Scalar(0, 255, 255);
const cv::Scalar YoloNet::RED = cv::Scalar(0, 0, 255);

/********************************************************************************
* Calibration definitions
********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: YoloNet
 * Description: Class constructor
 ********************************************************************************/
YoloNet::YoloNet(void) {};

/********************************************************************************
 * Function: ~YoloNet
 * Description: Class destructor
 ********************************************************************************/
YoloNet::~YoloNet(void) {};

/********************************************************************************
 * Function: create
 * Description: Class destructor
 ********************************************************************************/
cv::dnn::Net YoloNet::create(const std::string& model, const std::string& class_list_path, int backend_id, int target_id) 
{
    cv::dnn::Net net;

    // Create an ifstream to open the class list file
    std::ifstream class_labels(class_list_path);
    if (!class_labels.is_open()) 
    {
        std::cerr << "Error: Could not open file " << class_list_path << std::endl;
        return net; // Return an empty network if the file cannot be opened
    }

    // Clear the existing class labels before adding new ones
    class_list.clear();

    // Read each line from the file and add to class_list vector
    std::string line;
    while (std::getline(class_labels, line))
    {
        class_list.push_back(line);
    }

    // Create and configure the network
    net = cv::dnn::readNet(model);
    net.setPreferableBackend(backend_id);
    net.setPreferableTarget(target_id);

    return net;
}

/********************************************************************************
 * Function: draw_label
 * Description: Draw bounding box around detected object.
 ********************************************************************************/
void YoloNet::draw_label(cv::Mat& input, std::string label, int left, int top)
{
    // Display the label at the top of the bounding box.
    int baseLine;
    cv::Size label_size = cv::getTextSize(label, YoloNet::FONT_FACE, YoloNet::FONT_SCALE, YoloNet::THICKNESS, &baseLine);
    top = std::max(top, label_size.height);
    // Top left corner.
    cv::Point tlc = cv::Point(left, top);
    // Bottom right corner.
    cv::Point brc = cv::Point(left + label_size.width, top + label_size.height + baseLine);
    // Draw white rectangle.
    cv::rectangle(input, tlc, brc, YoloNet::BLACK, cv::FILLED);
    // Put the label on the black rectangle.
    cv::putText(input, label, cv::Point(left, top + label_size.height), YoloNet::FONT_FACE, YoloNet::FONT_SCALE, YoloNet::YELLOW, YoloNet::THICKNESS);
}

/********************************************************************************
 * Function: pre_process
 * Description: Process image and return raw detection outputs.
 ********************************************************************************/
std::vector<cv::Mat> YoloNet::pre_process(cv::Mat& input, cv::dnn::Net& net)
{
    // Convert to blob.
    cv::Mat blob;
    cv::dnn::blobFromImage(input, blob, 1. / 255., cv::Size(YoloNet::INPUT_WIDTH, YoloNet::INPUT_HEIGHT), cv::Scalar(), true, false);

    net.setInput(blob);

    // Forward propagate.
    std::vector<cv::Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());

    return outputs;
}

void YoloNet::clear_prev_detections(std::vector<YoloNet::detection>& detections) 
{
    detections.clear();
}

void YoloNet::unwrap_detections(float* data, int rows, float x_factor, float y_factor,
    std::vector<int>& class_ids, std::vector<float>& confidences,
    std::vector<cv::Rect>& boxes, std::vector<YoloNet::detection>& detections,
    const std::vector<std::string>& class_list) 
{
    const int dimensions = 85;

    for (int i = 0; i < rows; ++i) {
        float confidence = data[4];
        if (confidence >= YoloNet::CONFIDENCE_THRESHOLD) {
            float* classes_scores = data + 5;
            cv::Mat scores(1, class_list.size(), CV_32FC1, classes_scores);
            cv::Point class_id;
            double max_class_score;
            cv::minMaxLoc(scores, 0, &max_class_score, 0, &class_id);

            if (max_class_score > YoloNet::SCORE_THRESHOLD) {
                confidences.push_back(confidence);
                class_ids.push_back(class_id.x);

                float cx = data[0];
                float cy = data[1];
                float w = data[2];
                float h = data[3];

                int left = int((cx - 0.5 * w) * x_factor);
                int top = int((cy - 0.5 * h) * y_factor);
                int width = int(w * x_factor);
                int height = int(h * y_factor);

                boxes.push_back(cv::Rect(left, top, width, height));

                // Populate the detection structure
                YoloNet::detection det;
                det.ClassID = static_cast<uint32_t>(class_id.x);
                det.Confidence = confidence;
                det.Left = left;
                det.Top = top;
                det.Right = left + width;
                det.Bottom = top + height;

                // Add to detections vector
                detections.push_back(det);
            }
        }
        data += dimensions;
    }
}


void YoloNet::nms_suppression(const std::vector<cv::Rect>& boxes, const std::vector<float>& confidences,
    std::vector<int>& indices) 
{
    cv::dnn::NMSBoxes(boxes, confidences, YoloNet::SCORE_THRESHOLD, YoloNet::NMS_THRESHOLD, indices);
}

void YoloNet::draw_bounding_boxes(cv::Mat& input, const std::vector<int>& indices,
    const std::vector<cv::Rect>& boxes, const std::vector<float>& confidences,
    const std::vector<int>& class_ids, const std::vector<std::string>& class_list,
    std::vector<YoloNet::detection>& detections) 
{
    for (int i = 0; i < indices.size(); i++) 
    {
        int idx = indices[i];
        cv::Rect box = boxes[idx];
        int left = box.x;
        int top = box.y;
        int width = box.width;
        int height = box.height;

        // Draw bounding box
        cv::rectangle(input, cv::Point(left, top), cv::Point(left + width, top + height), YoloNet::BLUE, 3 * YoloNet::THICKNESS);

        // Get the label for the class name and its confidence
        std::string label = cv::format("%.2f", confidences[idx]);
        label = class_list[class_ids[idx]] + ":" + label;

        // Draw class labels
        YoloNet::draw_label(input, label, left, top);

        // Update the detection with NMS results
        YoloNet::detection& det = detections[idx];
        det.Left = left;
        det.Top = top;
        det.Right = left + width;
        det.Bottom = top + height;
    }
}


/********************************************************************************
 * Function: post_process
 * Description: Process image and raw detections.
 ********************************************************************************/
void YoloNet::post_process(cv::Mat& input, std::vector<cv::Mat>& outputs, std::vector<YoloNet::detection>& detections, const std::vector<std::string>& class_list)
{
    // Clear the previous detections
    clear_prev_detections(detections);

    // Initialize vectors to hold respective outputs while unwrapping detections.
    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    // Resizing factor
    float x_factor = input.cols / YoloNet::INPUT_WIDTH;
    float y_factor = input.rows / YoloNet::INPUT_HEIGHT;

    float* data = (float*)outputs[0].data;
    const int rows = 25200;

    // Unwrap detections from the output data
    unwrap_detections(data, rows, x_factor, y_factor, class_ids, confidences, boxes, detections, class_list);

    // Perform Non-Maximum Suppression
    std::vector<int> indices;
    nms_suppression(boxes, confidences, indices);

    // Draw bounding boxes and labels
    draw_bounding_boxes(input, indices, boxes, confidences, class_ids, class_list, detections);
}


/********************************************************************************
 * Function: detect_objects
 * Description: Initialize the network used for object detection.
 ********************************************************************************/
void YoloNet::detect(cv::Mat& input, cv::dnn::Net net, std::vector<YoloNet::detection>& detections)
{
    outputs = YoloNet::pre_process(input, net);
    YoloNet::post_process(input, outputs, detections, class_list);
}

#endif // BLD_WIN
#endif // ENABLE_CV
