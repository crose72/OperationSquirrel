# require CMake 2.8 or greater
cmake_minimum_required(VERSION 2.8)

# Define the configuration flag
# Use UART functionality on Jetson Nano
# Use TCP functionality
# add_definitions(-DUSE_TCP)  # Uncomment this line to use TCP
add_definitions(-DUSE_UART)  # Uncomment this line to use UART

# Include the "c" folder and its subfolders
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/c)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/c/calibration)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/c/experimental)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/c/plugins)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/c/tracking)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/apphdr)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/mavlink/v2.0/common)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/apphdr)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/tests)

# declare main project
project(main)

# import jetson-inference and jetson-utils packages
find_package(jetson-utils)
find_package(jetson-inference)

# find VPI package (optional)
find_package(VPI 2.0)

# CUDA is required
find_package(CUDA)

# add directory for libnvbuf-utils to program
link_directories(/usr/lib/aarch64-linux-gnu/tegra)

# Set the source directory
set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/appsrc)

# Set the header directory
set(HEADER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/apphdr)

# List all source files with .cpp extension
file(GLOB SOURCES ${SOURCE_DIR}/*.cpp)

# List all header files with .h extension
file(GLOB HEADERS ${HEADER_DIR}/*.h)


# List your source files
#set(SOURCES
#    main.cpp
#    videoIO.cpp
#	target_tracking.cpp
#)

# List your header files
#set(HEADERS
#	target_tracking.h
#    videoIO.h
#)

# compile the main program
cuda_add_executable(main ${SOURCES} ${HEADERS})

# link main to jetson-inference library
target_link_libraries(main jetson-inference)

target_link_libraries(main jsoncpp)

