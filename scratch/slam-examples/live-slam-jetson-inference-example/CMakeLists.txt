# require CMake 2.8 or greater
cmake_minimum_required(VERSION 2.8)

# Include the "c" folder and its subfolders
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/c)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/c/calibration)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/c/experimental)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/c/plugins)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/c/tracking)

include_directories(
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

# declare my-recognition project
project(detectnet)

# import jetson-inference and jetson-utils packages.
# note that if you didn't do "sudo make install"
# while building jetson-inference, this will error.
find_package(jetson-utils)
find_package(jetson-inference)

# find VPI package (optional)
find_package(VPI 2.0)

# CUDA is required
find_package(CUDA)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Find Pangolin
find_package(Pangolin REQUIRED)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# add directory for libnvbuf-utils to program
link_directories(/usr/lib/aarch64-linux-gnu/tegra)

# Set the source and header directories
set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(HEADER_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# List all source files with .cpp extension
file(GLOB SOURCES ${SOURCE_DIR}/*.cpp)
file(GLOB HEADERS ${HEADER_DIR}/*.h)

# compile the my-recognition program
cuda_add_executable(main ${SOURCES} ${HEADERS})

# link my-recognition to jetson-inference library
target_link_libraries(main 
    jetson-inference  
    ${OpenCV_LIBS} 
    ${Pangolin_LIBRARIES} 
    ORB_SLAM3)

