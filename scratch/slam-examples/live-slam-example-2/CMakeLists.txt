cmake_minimum_required(VERSION 3.10)
project(main)

option(USE_JETSON "Enable Jetson Nano specific features" ON)

find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(jetson-utils)
find_package(jetson-inference)
find_package(VPI 2.0)
find_package(CUDA)

find_library(GST_WEBRTC_LIB NAMES gstwebrtc-1.0 PATHS /usr/lib/aarch64-linux-gnu/gstreamer-1.0)
if(GST_WEBRTC_LIB)
    message(STATUS "Found GStreamer WebRTC library: ${GST_WEBRTC_LIB}")
else()
    message(FATAL_ERROR "GStreamer WebRTC library not found")
endif()

# Add include directories
include_directories(
    /usr/local/include/jetson-inference
    /usr/local/include/jetson-utils
    ${OpenCV_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    /usr/include/eigen3
    /usr/local/include
    /usr/local/include/sigslot
    /usr/local/include/pangolin
    /usr/local/include/ORB_SLAM3
    /usr/local/include/ORB_SLAM3/include
    /usr/local/include/ORB_SLAM3/include/CameraModels
    /usr/local/include/Thirdparty/Sophus
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/apphdr)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/tests)

set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/appsrc)
set(HEADER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/apphdr)

file(GLOB SOURCES ${SOURCE_DIR}/*.cpp)
file(GLOB HEADERS ${HEADER_DIR}/*.h)

link_directories(/usr/lib/aarch64-linux-gnu/tegra)
link_directories(/usr/lib/aarch64-linux-gnu/gstreamer-1.0)

cuda_add_executable(main ${SOURCES} ${HEADERS})

target_link_libraries(main 
                    ${OpenCV_LIBS} 
                    ${Pangolin_LIBRARIES} 
                    ORB_SLAM3
                    jetson-inference
                    jetson-utils)
