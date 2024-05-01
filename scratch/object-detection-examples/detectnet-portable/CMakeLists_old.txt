# require CMake 2.8 or greater
cmake_minimum_required(VERSION 2.8)

# Include the "c" folder and its subfolders
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Get a list of all the .cpp files in the current directory and subdirectories
file(GLOB_RECURSE CPP_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

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

# add directory for libnvbuf-utils to program
link_directories(/usr/lib/aarch64-linux-gnu/tegra)

# compile the my-recognition program
cuda_add_executable(detectnet ${CPP_FILES})

# link my-recognition to jetson-inference library
target_link_libraries(detectnet jetson-inference)

