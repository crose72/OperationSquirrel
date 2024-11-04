cmake_minimum_required(VERSION 2.8)

# Project definition
project(SquirrelDefender)

# Choose ONLY ONE platform for the build 
option(JETSON_B01 "Compiling for Jetson Nano B01" ON)
option(WSL "Compiling for WSL" OFF)
option(TCP "Compiling for Windows simulated environment" OFF)
option(SERIAL "Compiling for Windows with connected physical devices" OFF)
option(ENABLE_CV "Enable video and computer vision functionality" ON)

# Ensure only one build platform is selected
set(BUILD_PLATFORMS 0)

if(JETSON_B01)
    math(EXPR BUILD_PLATFORMS "${BUILD_PLATFORMS} + 1")
endif()

if(WSL)
    math(EXPR BUILD_PLATFORMS "${BUILD_PLATFORMS} + 1")
endif()

if(WIN32)
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

if(POLICY CMP0146)
    cmake_policy(SET CMP0146 OLD)  # Allows the use of find_package(CUDA)
endif()


# Define valid build types
set(VALID_BUILD_TYPES Debug Release)

# Validate CMAKE_BUILD_TYPE
if(NOT MSVC)
    if(NOT CMAKE_BUILD_TYPE)
        message(FATAL_ERROR "Invalid build type: ${CMAKE_BUILD_TYPE}. Choose either Debug or Release.")
    endif()
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

if (JETSON_B01)
    add_definitions(-DENABLE_CV)
    message("Enabled video and computer vision functionality.")

elseif(WIN32)
    if (ENABLE_CV)
        add_definitions(-DENABLE_CV)
        message("Enabled video and computer vision functionality.")
    else()
        message("Disabled video and computer vision functionality.")
    endif()
endif()

# Optionally include code that is specific to different platforms
if (JETSON_B01)
    add_definitions(-DJETSON_B01)
    add_definitions(-DJETSON_B01_SERIAL)
    message("Building for JETSON_B01 with serial comms.")
elseif(WSL)
    add_definitions(-DWSL)
    add_definitions(-DWSL_TCP)
    message("Building for WSL with TCP comms.")
elseif(WIN32 AND TCP)
    add_definitions(-DSIM_VEH)
    add_definitions(-DWIN32_TCP)
    message("Building for _WIN32 with TCP comms.")
elseif(WIN32 and SERIAL)
    add_definitions(-DREAL_VEH)
    add_definitions(-DWIN32_SERIAL)
    message("Building for _WIN32 with serial comms.")
else()
    message(FATAL_ERROR "Invalid build options.")
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
                        ${OpenCV_INCLUDE_DIRS})

    # Set CUDA paths
    set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda")
    set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};${CUDA_TOOLKIT_ROOT_DIR}")
    message(STATUS "CUDA_TOOLKIT_ROOT_DIR is set to ${CUDA_TOOLKIT_ROOT_DIR}")
    message(STATUS "CMAKE_PREFIX_PATH is set to ${CMAKE_PREFIX_PATH}")

    #message(STATUS "Attempting to find VPI 2.0...")
    #set(VPI_DIR "/opt/nvidia/vpi1")
    #find_package(VPI 1.2 REQUIRED)

    message(STATUS "Attempting to find CUDA...")
    find_package(CUDA REQUIRED)

    find_package(OpenCV REQUIRED)
    find_package(jetson-utils REQUIRED)
    find_package(jetson-inference REQUIRED)
    message(STATUS "Attempting to find JetsonGPIO library...")
    find_library(JETSON_GPIO_LIB JetsonGPIO)

    link_directories(/usr/lib/aarch64-linux-gnu/tegra)
elseif (WIN32)
    include_directories(
        C:/TensorRT/TensorRT-10.4.0.26/include
        C:/opencv/opencv-4.10.0/install/include
    )

    link_directories(
        C:/TensorRT/TensorRT-10.4.0.26/lib
        C:/opencv/opencv-4.10.0/install/x64/vc17/lib
        "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v12.6/lib/x64"
    )

    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Release>:Release>")
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
                        ${JETSON_GPIO_LIB}
                        jetson-inference
                        jetson-utils
                        jsoncpp)
elseif(WSL)
    add_executable(squirreldefender ${SOURCES} ${HEADERS})
elseif(WIN32)
    add_executable(squirreldefender ${SOURCES} ${HEADERS})
    target_link_libraries(squirreldefender Ws2_32)

    add_compile_definitions($<$<CONFIG:Debug>:_ITERATOR_DEBUG_LEVEL=2>)
    add_compile_definitions($<$<CONFIG:Release>:_ITERATOR_DEBUG_LEVEL=0>)

    target_link_libraries(squirreldefender 
        opencv_world4100
    )

    target_link_libraries(squirreldefender
        cudart_static
        cublas
        cublasLt
        cudadevrt
        cuda
        cufft
        cufftw
        cufilt
        curand
        cusolver
        cusolverMg
        cusparse
        nppc
        nppial
        nppicc
        nppidei
        nppif
        nppig
        nppim
        nppist
        nppisu
        nppitc
        npps
        nvblas
        nvfatbin
        nvfatbin_static
        nvJitLink
        nvJitLink_static
        nvjpeg
        nvml
        nvptxcompiler_static
        nvrtc
        nvrtc_static
        nvrtc-builtins_static
        OpenCL
    )

    target_link_libraries(squirreldefender
        nvinfer_10
        nvinfer_plugin_10
        nvonnxparser_10
    )

    target_link_libraries(squirreldefender
        cudnn
        cudnn64_8
        cudnn_adv_infer
        cudnn_adv_infer64_8
        cudnn_adv_train
        cudnn_adv_train64_8
        cudnn_cnn_infer
        cudnn_cnn_infer64_8
        cudnn_cnn_train
        cudnn_cnn_train64_8
        cudnn_ops_infer
        cudnn_ops_infer64_8
        cudnn_ops_train
        cudnn_ops_train64_8
    )

    set_target_properties(squirreldefender PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
endif()
