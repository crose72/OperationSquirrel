# Require CMake 3.1 or greater for IN_LIST support, 2.8 is for detectnet (Currently using CMake 3.28)
cmake_minimum_required(VERSION 3.1)

# Project definition
project(SquirrelDefender)

# Choose ONLY ONE platform for the build 
option(JETSON_B01 "Compiling for Jetson Nano B01" ON)
option(WSL "Compiling for WSL" OFF)

# Ensure only one build platform is selected
set(BUILD_PLATFORMS 0)
if(JETSON_B01)
    math(EXPR BUILD_PLATFORMS "${BUILD_PLATFORMS} + 1")
endif()

if(WSL)
    math(EXPR BUILD_PLATFORMS "${BUILD_PLATFORMS} + 1")
endif()

if(BUILD_PLATFORMS GREATER 1)
    message(FATAL_ERROR "Error: More than one build platform option is selected. Please select only one of the following options: JETSON_B01, WSL, or other supported options.")
endif()

# Set the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Set the policy for IN_LIST support
if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()

# Define valid build types
set(VALID_BUILD_TYPES Debug Release)

# Validate CMAKE_BUILD_TYPE
if(NOT CMAKE_BUILD_TYPE IN_LIST VALID_BUILD_TYPES)
  message(FATAL_ERROR "Invalid build type: ${CMAKE_BUILD_TYPE}. Choose either Debug or Release.")
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Compiler flags for each build type
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG -g")

# Optional include code into debug or release build
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DDEBUG_BUILD)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_definitions(-DRELEASE_BUILD)
endif()

# Optionally include code compiled for Jetson or WSL
if (JETSON_B01)
    add_definitions(-DJETSON_B01)
    add_definitions(-DUSE_UART)
elseif(WSL)
    add_definitions(-DWSL)
    add_definitions(-DUSE_TCP)
endif()

# Directories common to all builds
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/apphdr)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/tests)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/inc/mavlink/v2.0/common)

# Jetson B01 specific configurations
if (JETSON_B01)

    include_directories(/usr/local/include/JetsonGPIO
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
                        /usr/local/include/Thirdparty/Sophus)

    find_package(OpenCV REQUIRED)
    find_package(Pangolin REQUIRED)
    find_package(Eigen3 REQUIRED)
    find_package(jetson-utils REQUIRED)
    find_package(jetson-inference REQUIRED)
    find_package(VPI 2.0)
    find_package(CUDA REQUIRED)
    find_library(JETSON_GPIO_LIB JetsonGPIO)

    link_directories(/usr/lib/aarch64-linux-gnu/tegra)
endif()

# squirreldefender and source file paths
set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/appsrc)
set(HEADER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/apphdr)

# List all source files with .cpp extension
file(GLOB SOURCES ${SOURCE_DIR}/*.cpp)
file(GLOB HEADERS ${HEADER_DIR}/*.h)

# Compile the squirreldefender program
if (JETSON_B01)
    cuda_add_executable(squirreldefender ${SOURCES} ${HEADERS})

    target_link_libraries(squirreldefender 
                        ${OpenCV_LIBS} 
                        ${Pangolin_LIBRARIES} 
                        ${JETSON_GPIO_LIB}
                        ORB_SLAM3
                        jetson-inference
                        jetson-utils
                        jsoncpp)
elseif(WSL)
    add_executable(squirreldefender ${SOURCES} ${HEADERS})
endif()