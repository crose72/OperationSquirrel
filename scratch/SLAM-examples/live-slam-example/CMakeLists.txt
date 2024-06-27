cmake_minimum_required(VERSION 3.10)
project(main)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Find Pangolin
find_package(Pangolin REQUIRED)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Add include directories
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

# Add executable
add_executable(main main.cc)

# Specify the folder containing your .cc files
#file(GLOB SOURCES "../ORB_SLAM3/src/*.cc")

# Add the source files to the executable
#target_sources(main PRIVATE ${SOURCES})

# Link libraries
target_link_libraries(main ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ORB_SLAM3)
