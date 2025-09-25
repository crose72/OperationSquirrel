cmake_minimum_required(VERSION 3.10)

# Project definition
project(SquirrelDefender)

#=============================================================================
# Environment Variables Used (when USE_ENV_VAR=ON):
#
# Windows Build:
#   Library Paths (*.lib, *.dll):
#   LIB_TENSORRT    - TensorRT libraries (e.g., C:/TensorRT/TensorRT-10.4.0.26/lib)
#   LIB_OPENCV      - OpenCV libraries (e.g., C:/opencv/opencv-4.10.0/install/x64/vc17/lib)
#   LIB_CUDA        - CUDA libraries (e.g., C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v12.6/lib/x64)
#
#   Header Paths (*.h, *.hpp):
#   HDR_TENSORRT    - TensorRT headers (e.g., C:/TensorRT/TensorRT-10.4.0.26/include)
#   HDR_OPENCV      - OpenCV headers (e.g., C:/opencv/opencv-4.10.0/install/include)
#   HDR_CUDA        - CUDA headers (e.g., C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v12.6/include)
#
# Jetson Build:
#   Library Paths (*.so):
#   LIB_JETSONGPIO       - JetsonGPIO libraries (e.g., /usr/local/lib)
#   LIB_JETSONINFERENCE  - jetson-inference libraries (e.g., /usr/local/lib)
#   LIB_JETSONUTILS      - jetson-utils libraries (e.g., /usr/local/lib)
#   LIB_OPENCV           - OpenCV libraries (e.g., /usr/local/lib)
#   LIB_CUDA            - CUDA libraries (e.g., /usr/local/cuda/lib64)
#   LIB_TEGRA           - Tegra-specific libraries (e.g., /usr/lib/aarch64-linux-gnu/tegra)
#
#   Header Paths (*.h, *.hpp):
#   HDR_JETSONGPIO       - JetsonGPIO headers (e.g., /usr/local/include/JetsonGPIO)
#   HDR_JETSONINFERENCE  - jetson-inference headers (e.g., /usr/local/include/jetson-inference)
#   HDR_JETSONUTILS      - jetson-utils headers (e.g., /usr/local/include/jetson-utils)
#   HDR_OPENCV           - OpenCV headers (e.g., /usr/local/include/opencv4)
#   HDR_CUDA            - CUDA headers (e.g., /usr/local/cuda/include)
#
# Usage:
#   cmake -DUSE_ENV_VAR=ON ..
#
# Notes:
#   - Windows paths use forward slashes (/) for CMake compatibility
#   - Library paths should point to directories containing the binary files
#   - Header paths should point to directories containing the include files
#   - Some libraries might have additional platform-specific paths:
#     - Jetson: /usr/lib/aarch64-linux-gnu/tegra
#     - Windows: Additional DLL dependencies might be in System32
#=============================================================================

#=============================================================================
# Build Options (manually choose the options for your build)
#=============================================================================

# Choose ONLY ONE platform for the build (BLD_JETSON_B01, BLD_WSL, BLD_WIN) 
option(BLD_JETSON_B01 "Compiling for Jetson Nano B01, with video and computer vision functionality" OFF)
option(BLD_JETSON_ORIN_NANO "Compiling for Jetson Orin Nano, with video and computer visiono functionality" OFF)
option(BLD_WSL "Compiling for BLD_WSL" ON)
option(BLD_WIN "Compiling for Windows environment" OFF)
option(WIN_TCP "Compiling for Windows simulated environment" OFF)
option(WIN_SERIAL "Compiling for Windows with connected physical devices" OFF)
option(WIN_ENABLE_CV "Compiling for Windows with video and computer vision functionality" OFF)
option(BUILD_TEST_HARNESS "Compiling test harness code for data playback" ON)
option(ENABLE_GDB "Enable GDB debugger - turn OFF for release/production builds" OFF)
option(ENABLE_SECRET_SQUIRREL "Compile custom code with the open source Operation Squirrel" ON)

# Option to use environment variables for library paths
option(USE_ENV_VAR "Use environment variables for library paths" OFF)

#=============================================================================
# Verify Build Combinations are Valid
#=============================================================================

# Ensure only one build platform is selected
set(BUILD_PLATFORMS 0)

if(BLD_JETSON_ORIN_NANO)
    math(EXPR BUILD_PLATFORMS "${BUILD_PLATFORMS} + 1")
endif()

if(BLD_JETSON_B01)
    math(EXPR BUILD_PLATFORMS "${BUILD_PLATFORMS} + 1")
endif()

if(BLD_WSL)
    math(EXPR BUILD_PLATFORMS "${BUILD_PLATFORMS} + 1")
endif()

if(BLD_WIN)
    math(EXPR BUILD_PLATFORMS "${BUILD_PLATFORMS} + 1")
endif()

if(BUILD_PLATFORMS GREATER 1)
    message(FATAL_ERROR "Error: More than one build platform option is selected. Please select only one of the following options: BLD_JETSON_B01, BLD_WSL, or other supported options.")
endif()

#=============================================================================
# CMAKE specific settings
#=============================================================================

# Set C++ version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Set the policy for IN_LIST support (since we are using the IN_LIST functionality in cmake)
if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()

# Allow using find_package(CUDA)
if(POLICY CMP0146)
    cmake_policy(SET CMP0146 OLD)
endif()

# CMAKE modules path for Find* files and other cmake specific things
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

#=============================================================================
# Debugging Settings
#=============================================================================

# Validate CMAKE_BUILD_TYPE
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
    message(STATUS "Build type is: ${CMAKE_BUILD_TYPE}. Specify Debug if desired.")
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Function to set compilation flags for specific target
function(set_target_debug_flags target)
    if(MSVC)
        # For MSVC, control debug info and optimization per target
        target_compile_options(${target} PRIVATE 
            $<$<CONFIG:Debug>:/Zi /Od>
            $<$<CONFIG:Release>:/O2>)
        
        # Set linker flags to generate PDB files
        set_target_properties(${target} PROPERTIES 
            LINK_FLAGS_DEBUG "/DEBUG /INCREMENTAL"
            LINK_FLAGS_RELEASE "/INCREMENTAL:NO")
    else()
        # For GCC/Clang
        if (ENABLE_GDB)
            target_compile_options(${target} PRIVATE
                $<$<CONFIG:Debug>:-g -O0>
                $<$<CONFIG:Release>:-g -O3>)
        else()
            target_compile_options(${target} PRIVATE
                $<$<CONFIG:Debug>:-g -O0>
                $<$<CONFIG:Release>:-O3>)
        endif()
    endif()
endfunction()

# Enable ASan

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer -g")
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address")

#=============================================================================
# Define Preprocessor Directives Based on Build Options
#=============================================================================

# Optionally include code that is specific to different platforms
if (BLD_JETSON_B01)
    add_definitions(-DBLD_JETSON_B01)
    add_definitions(-DBLD_LINUX_SERIAL)
    message("Building for BLD_JETSON_B01 with serial comms.")
    add_definitions(-DENABLE_CV)
    message("Enabled video and computer vision functionality.")
elseif (BLD_JETSON_ORIN_NANO)
    add_definitions(-DBLD_JETSON_ORIN_NANO)
    add_definitions(-DBLD_LINUX_SERIAL)
    message("Building for BLD_JETSON_ORIN_NANO with serial comms.")
    add_definitions(-DENABLE_CV)
    message("Enabled video and computer vision functionality.")
elseif(BLD_WSL)
    add_definitions(-DBLD_WSL)
    add_definitions(-DBLD_WSL_TCP)
    message("Building for BLD_WSL with TCP comms.")
    add_definitions(-DENABLE_CV)
    message("Enabled video and computer vision functionality.")
elseif(BLD_WIN AND WIN_TCP)
    add_definitions(-DSIM_VEH)
    add_definitions(-DBLD_WIN_TCP)
    add_definitions(-DBLD_WIN)
    message("Building for _BLD_WIN with TCP comms.")
    if (WIN_ENABLE_CV)
        add_definitions(-DENABLE_CV)
        message("Enabled video and computer vision functionality.")
    else()
        message("Disabled video and computer vision functionality.")
    endif()
elseif(BLD_WIN AND WIN_SERIAL)
    add_definitions(-DREAL_VEH)
    add_definitions(-DBLD_WIN_SERIAL)
    add_definitions(-DBLD_WIN)
    message("Building for _BLD_WIN with serial comms.")
    if (WIN_ENABLE_CV)
        add_definitions(-DENABLE_CV)
        message("Enabled video and computer vision functionality.")
    else()
        message("Disabled video and computer vision functionality.")
    endif()
else()
    message(FATAL_ERROR "Invalid build options.")
endif()

#=============================================================================
# Find External Libraries and Headers 
#=============================================================================

# Additional required packages to find
if(BLD_JETSON_B01)
    find_package(CUDA REQUIRED)
    find_package(OpenCV REQUIRED)
    find_package(jetson-utils REQUIRED)
    find_package(jetson-inference REQUIRED)
    find_library(JETSON_GPIO_LIB JetsonGPIO)
    find_package(Armadillo REQUIRED)
    find_package(LAPACK REQUIRED)
    find_package(BLAS REQUIRED)
elseif(BLD_JETSON_ORIN_NANO)
    find_package(CUDA REQUIRED)
    find_package(OpenCV REQUIRED)
    find_package(TensorRT REQUIRED)
    find_package(vpi 3.2 REQUIRED)
    find_package(Eigen3 REQUIRED)
    find_package(Armadillo REQUIRED)
    find_package(LAPACK REQUIRED)
    find_package(BLAS REQUIRED)
elseif(BLD_WSL)
    find_package(CUDA REQUIRED)
    find_package(OpenCV REQUIRED)
    find_package(TensorRT REQUIRED)
    #find_package(vpi 3.2 REQUIRED)
    find_package(Eigen3 REQUIRED)
    find_package(Armadillo REQUIRED)
    find_package(LAPACK REQUIRED)
    find_package(BLAS REQUIRED)
endif()

# TODO: Specify the path to TensorRT root dir
if (NOT TensorRT_DIR)
    set(TensorRT_DIR /home/cyrus/work/libs/TensorRT-10.0.0.6/)
endif()

# Set paths to acados installation
set(ACADOS_INSTALL_DIR /usr/local)
set(ACADOS_INCLUDE_DIR ${ACADOS_INSTALL_DIR}/include/acados)
set(ACADOS_LIB_DIR ${ACADOS_INSTALL_DIR}/lib)

# Environment variable validation and inclusion based on platform
if(USE_ENV_VAR)
    if(BLD_WIN)
        # Validate required Windows environment variables
        foreach(var LIB_TENSORRT LIB_OPENCV LIB_CUDA LIB_CUDNN HDR_TENSORRT HDR_OPENCV HDR_CUDA HDR_CUDNN)
            if(NOT DEFINED ENV{${var}})
                message(FATAL_ERROR "Environment variable ${var} is not defined")
            endif()
        endforeach()

        # Include Windows-specific directories from environment variables
        include_directories($ENV{HDR_TENSORRT})
        include_directories($ENV{HDR_OPENCV})
        include_directories($ENV{HDR_CUDA})
        include_directories($ENV{HDR_CUDNN})

        set(DIR_OPENCV $ENV{DIR_OPENCV})
        set(LIB_OPENCV $ENV{LIB_OPENCV})

        link_directories($ENV{LIB_TENSORRT})
        link_directories($ENV{LIB_OPENCV})
        link_directories($ENV{LIB_CUDA})
        link_directories($ENV{LIB_CUDNN})
    elseif(BLD_JETSON_B01)
        # Validate required Jetson environment variables
        foreach(var LIB_JETSONGPIO LIB_JETSONINFERENCE LIB_JETSONUTILS LIB_OPENCV LIB_CUDA LIB_TEGRA
                    HDR_JETSONGPIO HDR_JETSONINFERENCE HDR_JETSONUTILS HDR_OPENCV HDR_CUDA)
            if(NOT DEFINED ENV{${var}})
                message(FATAL_ERROR "Environment variable ${var} is not defined")
            endif()
        endforeach()

        # Include Jetson-specific directories from environment variables
        include_directories($ENV{HDR_JETSONGPIO})
        include_directories($ENV{HDR_JETSONINFERENCE})
        include_directories($ENV{HDR_JETSONUTILS})
        include_directories($ENV{HDR_OPENCV})
        include_directories($ENV{HDR_CUDA})

        link_directories($ENV{LIB_JETSONGPIO})
        link_directories($ENV{LIB_JETSONINFERENCE})
        link_directories($ENV{LIB_JETSONUTILS})
        link_directories($ENV{LIB_OPENCV})
        link_directories($ENV{LIB_CUDA})
        link_directories($ENV{LIB_TEGRA})
    elseif(BLD_JETSON_ORIN_NANO)
        # Validate required Jetson environment variables
        foreach(var LIB_JETSONINFERENCE LIB_JETSONUTILS LIB_OPENCV LIB_CUDA LIB_TEGRA
                    HDR_JETSONINFERENCE HDR_JETSONUTILS HDR_OPENCV HDR_CUDA)
            if(NOT DEFINED ENV{${var}})
                message(FATAL_ERROR "Environment variable ${var} is not defined")
            endif()
        endforeach()

        # Include Jetson-specific directories from environment variables
        include_directories($ENV{HDR_JETSONINFERENCE})
        include_directories($ENV{HDR_JETSONUTILS})
        include_directories($ENV{HDR_OPENCV})
        include_directories($ENV{HDR_CUDA})

        link_directories($ENV{LIB_JETSONINFERENCE})
        link_directories($ENV{LIB_JETSONUTILS})
        link_directories($ENV{LIB_OPENCV})
        link_directories($ENV{LIB_CUDA})
        link_directories($ENV{LIB_TEGRA})
    endif()
else()
    # Default paths when not using environment variables
    if(BLD_WIN)
        set(TENSORRT_INCLUDE_DIR "C:/TensorRT/TensorRT-10.4.0.26/include" CACHE PATH "Path to TensorRT include directory")
        
        include_directories(
            ${TENSORRT_INCLUDE_DIR}
            "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v12.6/include"
        )
        
        set(TENSORRT_LIB_DIR "C:/TensorRT/TensorRT-10.4.0.26/lib" CACHE PATH "Path to TensorRT library directory")
        
        link_directories(
            ${TENSORRT_LIB_DIR}
            "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v12.6/lib/x64"
        )
    elseif(BLD_JETSON_B01)
        include_directories(
            /usr/local/include/JetsonGPIO
            /usr/local/include/jetson-inference
            /usr/local/include/jetson-utils
            /usr/local/cuda/include
            /usr/local/include/opencv4
        )
        link_directories(
            /usr/local/lib
            /usr/local/cuda/lib64
            /usr/lib/aarch64-linux-gnu/tegra
        )

    elseif(BLD_JETSON_ORIN_NANO)
        include_directories(
            /usr/local/cuda/include
            /usr/include/opencv4
            ${EIGEN3_INCLUDE_DIR}
            ${ARMADILLO_INCLUDE_DIRS}
            ${ACADOS_INCLUDE_DIR}
            #/usr/local/include/blasfeo/include # needed for ACADOS
            #/usr/local/include/hpipm/include/ # needed for ACADOS
            #/usr/local/include/acados_c # needed for ACADOS
        )

        link_directories(
            /usr/local/lib
            /usr/local/cuda/lib64
            /usr/lib/aarch64-linux-gnu/tegra
            ${ARMADILLO_LIBRARY_DIRS}
            # ${ACADOS_LIB_DIR} # needed for ACADOS
        )
    elseif(BLD_WSL)
        include_directories(
            /usr/local/cuda/include
            /usr/include/opencv4
            ${EIGEN3_INCLUDE_DIR}
            ${ARMADILLO_INCLUDE_DIRS}
            ${ACADOS_INCLUDE_DIR}
            #/usr/local/include/blasfeo/include # needed for ACADOS
            #/usr/local/include/hpipm/include/ # needed for ACADOS
            #/usr/local/include/acados_c # needed for ACADOS
        )

        link_directories(
            /usr/local/lib
            /usr/local/cuda/lib64
            /usr/lib/aarch64-linux-gnu/tegra
            ${ARMADILLO_LIBRARY_DIRS}
            # ${ACADOS_LIB_DIR} # needed for ACADOS
        )
    endif()
endif()

#=============================================================================
# Libraries - Standalone code that is used in SquirrelDefender but is not the 
# "business logic".  Can be easily ported to another project or replaced
# with other suitable code.  They end up being included in the SOURCES that 
# build the program.
#=============================================================================

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libraries/mav/mavlink/v2.0/common)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libraries/mav)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libraries/serial)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libraries/math)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libraries/json)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libraries/print)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libraries/yolo)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libraries/pid)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libraries/timer)

file(GLOB MAV_UTILS_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/libraries/mav/*.cpp)
file(GLOB MAV_UTILS_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/libraries/mav/*.h)
file(GLOB SERIAL_UTILS_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/libraries/serial/*.cpp)
file(GLOB SERIAL_UTILS_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/libraries/serial/*.h)
file(GLOB MATH_UTILS_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/libraries/math/*.cpp)
file(GLOB MATH_UTILS_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/libraries/math/*.h)
file(GLOB JSON_UTILS_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/libraries/json/*.cpp)
file(GLOB JSON_UTILS_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/libraries/json/*.h)
file(GLOB YOLO_UTILS_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/libraries/yolo/*.cpp)
file(GLOB YOLO_UTILS_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/libraries/yolo/*.h)
file(GLOB PRINT_UTILS_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/libraries/print/*.cpp)
file(GLOB PRINT_UTILS_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/libraries/print/*.h)
file(GLOB PID_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/libraries/pid/*.cpp)
file(GLOB PID_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/libraries/pid/*.h)
file(GLOB TIMER_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/libraries/timer/*.cpp)
file(GLOB TIMER_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/libraries/timer/*.h)

#=============================================================================
# Modules - Other github repositories added as submodules that are used.
#=============================================================================

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/modules/kf/kalman)
file(GLOB KF_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/modules/kf/kalman/*.cpp)
file(GLOB KF_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/modules/kf/kalman/*.h)

if(ENABLE_SECRET_SQUIRREL)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/modules/SecretSquirrel/src)
    file(GLOB SS_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/modules/SecretSquirrel/src/*.cpp)
    file(GLOB SS_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/modules/SecretSquirrel/src/*.h)
endif()

if(BLD_JETSON_ORIN_NANO OR BLD_WSL)    
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/modules/trtinfer/src)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/modules/trtinfer/include)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/modules/trtinfer)
endif()

#=============================================================================
# Set SquirrelDefender Source files and locations
#=============================================================================

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/apphdr)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/test_flights)

# squirreldefender and source file paths
set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/appsrc)
set(HEADER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/apphdr)

# List all source files with .cpp extension
file(GLOB SOURCES ${SOURCE_DIR}/*.cpp)
file(GLOB HEADERS ${HEADER_DIR}/*.h)

# Additional source files to compile together
list(APPEND SOURCES ${MAV_UTILS_SOURCES} ${SERIAL_UTILS_SOURCES} ${MATH_UTILS_SOURCES} 
                    ${JSON_UTILS_SOURCES} ${PRINT_UTILS_SOURCES} ${YOLO_UTILS_SOURCES} 
                    ${PID_SOURCES} ${TIMER_SOURCES} ${KF_SOURCES} ${SS_SOURCES})
list(APPEND HEADERS ${MAV_UTILS_HEADERS} ${SERIAL_UTILS_HEADERS} ${MATH_UTILS_HEADERS} 
                    ${JSON_UTILS_HEADERS} ${PRINT_UTILS_HEADERS} ${YOLO_UTILS_HEADERS} 
                    ${PID_HEADERS} ${TIMER_HEADERS} ${KF_HEADERS} ${SS_HEADERS})

#=============================================================================
# Build Executable for each supported platform
#=============================================================================

# Build only the test harness or the full executable speeds up compile time (so worth it!)
if (NOT BUILD_TEST_HARNESS)
    message(STATUS "Building the squirreldefender program")  
    if (BLD_JETSON_B01)
        cuda_add_executable(squirreldefender ${SOURCES} ${HEADERS})
        set_target_debug_flags(squirreldefender)

        target_link_libraries(squirreldefender 
                            ${OpenCV_LIBS} 
                            ${JETSON_GPIO_LIB}
                            jetson-inference
                            jetson-utils
                            jsoncpp
                            ${ARMADILLO_LIBRARIES} 
                            ${LAPACK_LIBRARIES} 
                            ${BLAS_LIBRARIES})
    elseif (BLD_JETSON_ORIN_NANO)
        cuda_add_executable(squirreldefender ${SOURCES} ${HEADERS})
        set_target_debug_flags(squirreldefender)

        target_link_libraries(squirreldefender 
                            trtinfer 
                            ${OpenCV_LIBS} 
                            vpi 
                            opencv_core
                            opencv_imgproc 
                            opencv_imgcodecs 
                            opencv_videoio
                            jsoncpp
                            ${ARMADILLO_LIBRARIES} 
                            ${LAPACK_LIBRARIES} 
                            ${BLAS_LIBRARIES}
                            #acados     # For acados, remove if you are not using qpOASES
                            #hpipm      # For acados, remove if you are not using qpOASES
                            #blasfeo    # For acados, remove if you are not using qpOASES
                            #qpOASES_e  # For acados, remove if you are not using qpOASES
                            #osqp       # For acados, remove if you are not using OSQP
                            )

    elseif(BLD_WSL)
        cuda_add_executable(squirreldefender ${SOURCES} ${HEADERS})
        set_target_debug_flags(squirreldefender)

        target_link_libraries(squirreldefender 
                            trtinfer 
                            ${OpenCV_LIBS} 
                            #vpi 
                            opencv_core
                            opencv_imgproc 
                            opencv_imgcodecs 
                            opencv_videoio
                            jsoncpp
                            ${ARMADILLO_LIBRARIES} 
                            ${LAPACK_LIBRARIES} 
                            ${BLAS_LIBRARIES}
                            #acados     # For acados, remove if you are not using qpOASES
                            #hpipm      # For acados, remove if you are not using qpOASES
                            #blasfeo    # For acados, remove if you are not using qpOASES
                            #qpOASES_e  # For acados, remove if you are not using qpOASES
                            #osqp       # For acados, remove if you are not using OSQP
                            )
    elseif(BLD_WIN)

        if(USE_ENV_VAR)
            find_package(OpenCV REQUIRED
                PATHS $ENV{LIB_OPENCV}/../
                    $ENV{LIB_OPENCV}
                HINTS $ENV{LIB_OPENCV}/../
                    $ENV{LIB_OPENCV}
                NO_DEFAULT_PATH)  # Only look in specified paths
        else()
            find_package(OpenCV REQUIRED
                PATHS "C:/opencv/4.10.0/install"
                    "C:/opencv/4.10.0/install/x64/vc17"
                NO_DEFAULT_PATH)
        endif()

        # Find OpenCV package first
        find_package(OpenCV REQUIRED)
        message(STATUS "OpenCV library status:")
        message(STATUS "    version: ${OpenCV_VERSION}")
        message(STATUS "    libraries: ${OpenCV_LIBS}")
        message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

        add_executable(squirreldefender ${SOURCES} ${HEADERS})
        set_target_debug_flags(squirreldefender)
        
        # Add include directories
        target_include_directories(squirreldefender PRIVATE ${OpenCV_INCLUDE_DIRS})
        
        # Link with OpenCV first
        target_link_libraries(squirreldefender PRIVATE ${OpenCV_LIBS})
        
        # Add Windows sockets
        target_link_libraries(squirreldefender PRIVATE Ws2_32)

        # Set iterator debug level
        target_compile_definitions(squirreldefender PRIVATE 
            $<$<CONFIG:Debug>:_ITERATOR_DEBUG_LEVEL=2>
            $<$<CONFIG:Release>:_ITERATOR_DEBUG_LEVEL=0>)

        # Match the runtime library with OpenCV
        if(CMAKE_BUILD_TYPE STREQUAL "Debug")
            set(CMAKE_CXX_FLAGS_DEBUG "/Zi /Od /MDd")  # Use MDd for debug
            target_compile_options(squirreldefender PRIVATE /MDd)
        else()
            set(CMAKE_CXX_FLAGS_RELEASE "/O2 /DNDEBUG /MD")  # Use MD for release
            target_compile_options(squirreldefender PRIVATE /MD)
        endif()
        
        target_link_libraries(squirreldefender PRIVATE
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

        target_link_libraries(squirreldefender PRIVATE
            nvinfer_10
            nvinfer_plugin_10
            nvonnxparser_10
        )

        target_link_libraries(squirreldefender PRIVATE
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
endif()

#=============================================================================
# Build Test Harness for each supported platform
#=============================================================================

if (BUILD_TEST_HARNESS)
    message(STATUS "Building the test harness (not squirreldefender)")
    if (BLD_JETSON_ORIN_NANO OR BLD_WSL)

        # Test source base
        list(REMOVE_ITEM SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/appsrc/main.cpp")
        set(TEST_SOURCE_BASE ${SOURCES})
        file(GLOB TEST_UTIL_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/test_suite/test_utils/*.cpp)
        file(GLOB TEST_UTIL_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/test_suite/test_utils/*.h)
        list(APPEND TEST_SOURCE_BASE ${TEST_UTIL_SOURCES})
        include_directories(${CMAKE_CURRENT_SOURCE_DIR}/test_suite)

        # Localize target test code
        set(LOC_TEST_SOURCES ${TEST_SOURCE_BASE})
        list(APPEND LOC_TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/test_suite/test_localize_target.cpp)
        list(APPEND LOC_TEST_HEADERS ${TEST_UTIL_HEADERS})

        cuda_add_executable(test_localize_target ${LOC_TEST_SOURCES} ${LOC_TEST_HEADERS})
        set_target_debug_flags(test_localize_target)

        if (BLD_JETSON_ORIN_NANO)
            target_link_libraries(test_localize_target 
                                trtinfer 
                                ${OpenCV_LIBS} 
                                vpi 
                                opencv_core
                                opencv_imgproc 
                                opencv_imgcodecs 
                                opencv_videoio
                                jsoncpp
                                ${ARMADILLO_LIBRARIES} 
                                ${LAPACK_LIBRARIES} 
                                ${BLAS_LIBRARIES})
        elseif(BLD_WSL)
            target_link_libraries(test_localize_target 
                                trtinfer 
                                ${OpenCV_LIBS} 
                                #vpi 
                                opencv_core
                                opencv_imgproc 
                                opencv_imgcodecs 
                                opencv_videoio
                                jsoncpp
                                ${ARMADILLO_LIBRARIES} 
                                ${LAPACK_LIBRARIES} 
                                ${BLAS_LIBRARIES})
        endif()

        # Follow target test code
        set(FOLLOW_TEST_SOURCES ${TEST_SOURCE_BASE})
        list(APPEND FOLLOW_TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/test_suite/test_follow_target.cpp)
        list(APPEND FOLLOW_TEST_HEADERS ${TEST_UTIL_HEADERS})

        cuda_add_executable(test_follow_target ${FOLLOW_TEST_SOURCES} ${FOLLOW_TEST_HEADERS})
        set_target_debug_flags(test_follow_target)

        if (BLD_JETSON_ORIN_NANO)
            target_link_libraries(test_follow_target 
                                trtinfer 
                                ${OpenCV_LIBS} 
                                vpi 
                                opencv_core
                                opencv_imgproc 
                                opencv_imgcodecs 
                                opencv_videoio
                                jsoncpp
                                ${ARMADILLO_LIBRARIES} 
                                ${LAPACK_LIBRARIES} 
                                ${BLAS_LIBRARIES})
        elseif(BLD_WSL)
            target_link_libraries(test_follow_target 
                                trtinfer 
                                ${OpenCV_LIBS} 
                                #vpi 
                                opencv_core
                                opencv_imgproc 
                                opencv_imgcodecs 
                                opencv_videoio
                                jsoncpp
                                ${ARMADILLO_LIBRARIES} 
                                ${LAPACK_LIBRARIES} 
                                ${BLAS_LIBRARIES})
        endif()

        # Delivery test code
        set(DELIVERY_TEST_SOURCES ${TEST_SOURCE_BASE})
        list(APPEND DELIVERY_TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/modules/SecretSquirrel/test_suite/test_payload_delivery.cpp)
        list(APPEND DELIVERY_TEST_HEADERS ${TEST_UTIL_HEADERS})

        cuda_add_executable(test_payload_delivery ${DELIVERY_TEST_SOURCES} ${DELIVERY_TEST_HEADERS})
        set_target_debug_flags(test_payload_delivery)

        if (BLD_JETSON_ORIN_NANO)
            target_link_libraries(test_payload_delivery 
                                trtinfer 
                                ${OpenCV_LIBS} 
                                vpi 
                                opencv_core
                                opencv_imgproc 
                                opencv_imgcodecs 
                                opencv_videoio
                                jsoncpp
                                ${ARMADILLO_LIBRARIES} 
                                ${LAPACK_LIBRARIES} 
                                ${BLAS_LIBRARIES})
        elseif(BLD_WSL)
            target_link_libraries(test_payload_delivery 
                                trtinfer 
                                ${OpenCV_LIBS} 
                                #vpi 
                                opencv_core
                                opencv_imgproc 
                                opencv_imgcodecs 
                                opencv_videoio
                                jsoncpp
                                ${ARMADILLO_LIBRARIES} 
                                ${LAPACK_LIBRARIES} 
                                ${BLAS_LIBRARIES})
        endif()
    endif()
endif()
