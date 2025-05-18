cmake_minimum_required(VERSION 3.10)
project(video_scrubber)

find_package(OpenCV REQUIRED)

add_executable(video_scrubber main.cpp)
target_link_libraries(video_scrubber ${OpenCV_LIBS})
