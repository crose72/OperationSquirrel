#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: ./video_player <video_file>" << std::endl;
        return -1;
    }

    std::string video_path = argv[1];
    cv::VideoCapture cap(video_path);

    if (!cap.isOpened()) {
        std::cerr << "Error: Cannot open video file: " << video_path << std::endl;
        return -1;
    }

    double original_fps = cap.get(cv::CAP_PROP_FPS);
    std::cout << "Original video FPS: " << original_fps << std::endl;

    cv::Mat frame;
    int frame_count = 0;
    auto start_time = std::chrono::high_resolution_clock::now();

    while (true) {
        auto frame_start = std::chrono::high_resolution_clock::now();

        if (!cap.read(frame)) {
            std::cout << "End of video." << std::endl;
            break;
        }

        cv::imshow("Video Playback", frame);
        frame_count++;

        // WaitKey in ms based on original FPS
        int delay = static_cast<int>(1000.0 / original_fps);
        int key = cv::waitKey(delay);

        if (key == 27 || key == 'q') // ESC or 'q' to quit
            break;

        auto frame_end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = frame_end - frame_start;

        double actual_frame_time = elapsed.count();
        double actual_fps = 1.0 / actual_frame_time;

        std::cout << "Displayed frame " << frame_count
                  << " | Instant FPS: " << actual_fps << std::endl;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> total_elapsed = end_time - start_time;

    double avg_fps = frame_count / total_elapsed.count();
    std::cout << "Average playback FPS: " << avg_fps << std::endl;

    if (avg_fps > original_fps + 1.0) {
        std::cout << "Warning: Playback may be faster than real-time!" << std::endl;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
