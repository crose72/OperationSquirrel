#include <opencv2/opencv.hpp>
#include <iostream>

std::vector<cv::Rect> selected_regions;
bool drawing = false;
cv::Point pt1, pt2;

void mouse_handler(int event, int x, int y, int, void*) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        drawing = true;
        pt1 = pt2 = cv::Point(x, y);
    } else if (event == cv::EVENT_MOUSEMOVE && drawing) {
        pt2 = cv::Point(x, y);
    } else if (event == cv::EVENT_LBUTTONUP) {
        drawing = false;
        pt2 = cv::Point(x, y);
        cv::Rect region(pt1, pt2);
        if (region.area() > 0)
            selected_regions.push_back(region);
    }
}

std::string create_output_file_name(std::string input_path)
{
    // Find the last slash (folder separator)
    size_t last_slash = input_path.find_last_of("/\\");
    std::string directory = (last_slash != std::string::npos) ? input_path.substr(0, last_slash + 1) : "";

    // Get the filename part
    std::string filename = (last_slash != std::string::npos) ? input_path.substr(last_slash + 1) : input_path;

    // Find the last dot for extension
    size_t last_dot = filename.find_last_of(".");
    std::string name_only = (last_dot != std::string::npos) ? filename.substr(0, last_dot) : filename;
    std::string extension = (last_dot != std::string::npos) ? filename.substr(last_dot) : "";

    // Compose new output filename
    std::string output_path = directory + name_only + "_scrubbed" + extension;

    return output_path;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <input_video>" << std::endl;
        return 1;
    }

    std::string input_path = argv[1];
    std::string output_path = create_output_file_name(input_path);

    cv::VideoCapture cap(input_path);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open video: " << input_path << std::endl;
        return 1;
    }

    // Get video properties
    int width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    double fps = cap.get(cv::CAP_PROP_FPS);
    int fourcc = cv::VideoWriter::fourcc('m','p','4','v');  // or use 'X','V','I','D'

    // Grab first frame for region selection
    cv::Mat first_frame;
    cap >> first_frame;

    cv::namedWindow("Select Scrub Area");
    cv::setMouseCallback("Select Scrub Area", mouse_handler);

    while (true) {
        cv::Mat display = first_frame.clone();
        for (const auto& region : selected_regions) {
            cv::rectangle(display, region, cv::Scalar(0, 255, 0), 2);
        }
        if (drawing) {
            cv::rectangle(display, pt1, pt2, cv::Scalar(0, 255, 0), 2);
        }
        cv::imshow("Select Scrub Area", display);
        if (cv::waitKey(1) == 13) break; // Press ENTER to confirm
    }

    cv::destroyWindow("Select Scrub Area");

    // Rewind video
    cap.set(cv::CAP_PROP_POS_FRAMES, 0);

    // Setup writer
    cv::VideoWriter writer(output_path, fourcc, fps, cv::Size(width, height));
    if (!writer.isOpened()) {
        std::cerr << "Failed to open output video: " << output_path << std::endl;
        return 1;
    }

    // Process and save frames
    while (true) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) break;

        // Blur selected region
        for (const auto& region : selected_regions) {
            cv::Rect clipped = region & cv::Rect(0, 0, frame.cols, frame.rows);
            if (clipped.area() > 0) {
                cv::Mat roi = frame(clipped);
                cv::GaussianBlur(roi, roi, cv::Size(51, 51), 0);
            }
        }


        writer.write(frame);                // Save to file
        cv::imshow("Scrubbed Video", frame);
        if (cv::waitKey(1) == 'q') break;  // Press 'q' to quit early
    }

    cap.release();
    writer.release();
    cv::destroyAllWindows();

    std::cout << "Video saved to: " << output_path << std::endl;
    return 0;
}
