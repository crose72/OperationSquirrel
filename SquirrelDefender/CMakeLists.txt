# Require CMake 3.1 or greater for IN_LIST support, 2.8 is for detectnet
cmake_minimum_required(VERSION 3.1)

# Project definition
project(SquirrelDefender)

# Options to enable or disable features
option(USE_JETSON "Enable Jetson Nano specific features" ON)
option(USE_WSL "Enable WSL specific features" OFF)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Set the policy for IN_LIST support
if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()

# Define valid build types
set(VALID_BUILD_TYPES Debug Release)

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type (Debug or Release)")
endif()

# Validate CMAKE_BUILD_TYPE
if(NOT CMAKE_BUILD_TYPE IN_LIST VALID_BUILD_TYPES)
  message(FATAL_ERROR "Invalid build type: ${CMAKE_BUILD_TYPE}. Choose either Debug or Release.")
endif()

# Print the current build type
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Optional: Set specific compiler flags for each build type
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG -g")

# Define a macro for the build type
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DDEBUG_BUILD)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_definitions(-DRELEASE_BUILD)
endif()

# Conditional configurations for Jetson or WSL
if (USE_JETSON)
    option(USE_UART "Use UART functionality for Jetson Nano -> SITL or Jetson Nano -> real drone" ON)
    option(USE_TCP "Use TCP functionality for WSL -> WSL SITL" OFF)
    add_definitions(-DUSE_JETSON)
    add_definitions(-DUSE_UART)
elseif(USE_WSL)
    option(USE_UART "Use UART functionality for Jetson Nano -> SITL or Jetson Nano -> real drone" OFF)
    option(USE_TCP "Use TCP functionality for WSL -> WSL SITL" ON)
    add_definitions(-DUSE_WSL)
    add_definitions(-DUSE_TCP)
endif()

# Custom header includes
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/apphdr)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/tests)

# Mavlink includes
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/inc/mavlink/v2.0/common)

# Dynalo includes
include_directories(/usr/local/include/dynalo)
include_directories(/usr/local/include/dynalo/detail)
include_directories(/usr/local/include/dynalo/detail/linux)
include_directories(/usr/local/include/dynalo/detail/macos)
include_directories(/usr/local/include/dynalo/detail/windows)

# jetson-inference and utils includes
include_directories(/usr/local/include/jetson-inference)
include_directories(/usr/local/include/jetson-utils)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/c)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/c/calibration)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/c/experimental)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/c/plugins)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/c/tracking)

# NaturalSort includes
include_directories(/usr/local/include/NaturalSort)

# ORB_SLAM3 includes
include_directories(/usr/local/include/ORB_SLAM3)
include_directories(/usr/local/include/ORB_SLAM3/CameraModels)

# Pangolin includes
include_directories(/usr/local/include/pangolin)
include_directories(/usr/local/include/pangolin/compat)
include_directories(/usr/local/include/pangolin/console)
include_directories(/usr/local/include/pangolin/display)
include_directories(/usr/local/include/pangolin/factory)
include_directories(/usr/local/include/pangolin/geometry)
include_directories(/usr/local/include/pangolin/gl)
include_directories(/usr/local/include/pangolin/gl/compat)
include_directories(/usr/local/include/pangolin/handler)
include_directories(/usr/local/include/pangolin/image)
include_directories(/usr/local/include/pangolin/log)
include_directories(/usr/local/include/pangolin/plot)
include_directories(/usr/local/include/pangolin/plot/loaders)
include_directories(/usr/local/include/pangolin/scene)
include_directories(/usr/local/include/pangolin/tools)
include_directories(/usr/local/include/pangolin/utils)
include_directories(/usr/local/include/pangolin/utils/posix)
include_directories(/usr/local/include/pangolin/utils/xml)
include_directories(/usr/local/include/pangolin/var)
include_directories(/usr/local/include/pangolin/video)
include_directories(/usr/local/include/pangolin/video/drivers)
include_directories(/usr/local/include/pangolin/windowing)

# Sigslot includes
include_directories(/usr/local/include/sigslot)

# Thirdparty includes for ORB_SLAM3
include_directories(/usr/local/include/Thirdparty)
include_directories(/usr/local/include/Thirdparty/DBoW2)
include_directories(/usr/local/include/Thirdparty/DBoW2/DBoW2)
include_directories(/usr/local/include/Thirdparty/DBoW2/DUtils)
include_directories(/usr/local/include/Thirdparty/g2o)
include_directories(/usr/local/include/Thirdparty/g2o/g2o)
include_directories(/usr/local/include/Thirdparty/g2o/g2o/core)
include_directories(/usr/local/include/Thirdparty/g2o/g2o/solvers)
include_directories(/usr/local/include/Thirdparty/g2o/g2o/stuff)
include_directories(/usr/local/include/Thirdparty/g2o/g2o/types)
include_directories(/usr/local/include/Thirdparty/Sophus)
include_directories(/usr/local/include/Thirdparty/Sophus/sophus)

# Tinyobject includes
include_directories(/usr/local/include/tinyobj)

# Gstreamer includes
include_directories(/usr/include/gstreamer-1.0/)
include_directories(/usr/include/glib-2.0/)
include_directories(/usr/lib/aarch64-linux-gnu/glib-2.0/include/)

# GPIO includes
include_directories(/usr/local/include/JetsonGPIO)

# Jetson-specific configurations
if (USE_JETSON)
    # Import jetson-inference and jetson-utils packages
    find_package(jetson-utils REQUIRED)
    find_package(jetson-inference REQUIRED)
    
    # Find VPI package (optional)
    find_package(VPI 2.0)
    
    # CUDA is required
    find_package(CUDA REQUIRED)

    find_package(PkgConfig REQUIRED)
    pkg_check_modules(GST REQUIRED gstreamer-1.0)
    pkg_check_modules(GLIB REQUIRED glib-2.0)
    pkg_check_modules(GOBJECT REQUIRED gobject-2.0)
    pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)

    # Find the JetsonGPIO library
    find_library(JETSON_GPIO_LIB JetsonGPIO)

    # Include directories for GStreamer and GLib
    include_directories(${GST_INCLUDE_DIRS})
    include_directories(${GLIB_INCLUDE_DIRS})
    include_directories(${GOBJECT_INCLUDE_DIRS})
    include_directories(${GST_APP_INCLUDE_DIRS})
    link_directories(/usr/lib/aarch64-linux-gnu/tegra)

    # All .so files contained locally
    link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)
endif()

# Set the source and header directories
set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/appsrc)
set(HEADER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/apphdr)

# List all source files with .cpp extension
file(GLOB SOURCES ${SOURCE_DIR}/*.cpp)
file(GLOB TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/UnitTests/*.cpp)

# List all header files with .h extension
file(GLOB HEADERS ${HEADER_DIR}/*.h)

# Add the external directory for CppUTest
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../ThirdParty/CppUTest ThirdParty/CppUTest)
enable_testing()

# Create a static library for the squirreldefender code
add_library(squirrel_defender_lib STATIC ${SOURCES} ${HEADERS})

# Locate the static library
find_library(SQUIRREL_DEFENDER_LIB NAMES squirrel_defender_lib PATHS ${STATIC_LIBRARY_DIR})

# Compile the squirreldefender program
if (USE_JETSON)
    cuda_add_executable(squirreldefender ${SOURCES} ${HEADERS})
    target_link_libraries(squirreldefender
        jetson-inference
        jetson-utils
        jsoncpp
        g2o
        ORB_SLAM3
        pango_core
        pango_windowing
        pango_display
        pango_geometry
        pango_glgeometry
        pango_image
        pango_opengl
        pango_packetstream
        pango_plot
        pango_python
        pango_scene
        pango_tools
        pango_vars
        pango_video
        DBoW2
        libCatch2.a
        libCatch2Main.a
        tinyobj
        ${GST_LIBRARIES} 
        ${GLIB_LIBRARIES} 
        ${GOBJECT_LIBRARIES} 
        ${GST_APP_LIBRARIES}
        ${JETSON_GPIO_LIB})
elseif(USE_WSL)
    add_executable(squirreldefender ${SOURCES} ${HEADERS})
endif()

# Compile the unit_tests program
if (USE_JETSON)
    cuda_add_executable(unit_tests ${TEST_SOURCES} ${HEADERS})
    target_link_libraries(unit_tests jetson-inference jsoncpp)
    add_test(NAME unit_tests COMMAND unit_tests)
    target_link_libraries(unit_tests squirrel_defender_lib CppUTest CppUTestExt)
elseif(USE_WSL)
    add_executable(unit_tests ${TEST_SOURCES} ${HEADERS})
    add_test(NAME unit_tests COMMAND unit_tests)
    target_link_libraries(unit_tests squirrel_defender_lib CppUTest CppUTestExt)
endif()
