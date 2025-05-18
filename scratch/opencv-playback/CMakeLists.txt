cmake_minimum_required(VERSION 3.10)
project(video_player)

find_package(OpenCV REQUIRED)

add_executable(video_player main.cpp)
target_link_libraries(video_player ${OpenCV_LIBS})
