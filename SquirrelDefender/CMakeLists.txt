# Require CMake 3.1 or greater for IN_LIST support, 2.8 is for detectnet
cmake_minimum_required(VERSION 3.1)

# Options to enable or disable features.  Set these first.
option(USE_JETSON "Enable Jetson Nano specific features" ON)
option(USE_WSL "Enable WSL specific features" OFF)

# Set the policy for IN_LIST support
if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()

# Define valid build types
set(VALID_BUILD_TYPES Debug Release)

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type (Debug or Release)" FORCE)
endif()

# Validate CMAKE_BUILD_TYPE
if(NOT CMAKE_BUILD_TYPE IN_LIST VALID_BUILD_TYPES)
  message(FATAL_ERROR "Invalid build type: ${CMAKE_BUILD_TYPE}. Choose either Debug or Release.")
endif()

# Print the current build type
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Optional: Set specific compiler flags for each build type
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Define a macro for the build type
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DDEBUG_BUILD)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_definitions(-DRELEASE_BUILD)
endif()


if (USE_JETSON)
    project(SquirrelDefender)
    option(USE_UART "Use UART functionality for Jetson Nano -> SITL or Jetson Nano -> real drone" ON)
    option(USE_TCP "Use TCP functionality for WSL -> WSL SITL" OFF)
    add_definitions(-DUSE_JETSON)
    add_definitions(-DUSE_UART)
elseif(USE_WSL)
    project(SquirrelDefender)
    option(USE_UART "Use UART functionality for Jetson Nano -> SITL or Jetson Nano -> real drone" OFF)
    option(USE_TCP "Use TCP functionality for WSL -> WSL SITL" ON)
    add_definitions(-DUSE_WSL)
    add_definitions(-DUSE_TCP)
endif()

# Configuration flags
if (USE_TCP)
    add_definitions(-DUSE_TCP)
endif()

if (USE_UART)
    add_definitions(-DUSE_UART)
endif()

# Include the "c" folder and its subfolders
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/c)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/c/calibration)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/c/experimental)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/c/plugins)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/c/tracking)
include_directories(/usr/include/gstreamer-1.0/)
include_directories(/usr/include/glib-2.0/)
include_directories(/usr/lib/aarch64-linux-gnu/glib-2.0/include/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/apphdr)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/mavlink/v2.0/common)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/apphdr)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/tests)

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

    # Include directories for GStreamer and GLib
    include_directories(${GST_INCLUDE_DIRS})
    include_directories(${GLIB_INCLUDE_DIRS})
    include_directories(${GOBJECT_INCLUDE_DIRS})
    include_directories(${GST_APP_INCLUDE_DIRS})
    
    # Add directory for libnvbuf-utils to program
    link_directories(/usr/lib/aarch64-linux-gnu/tegra)
endif()

# Set the source directory
set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/appsrc)

# Set the header directory
set(HEADER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/apphdr)

# List all source files with .cpp extension
file(GLOB SOURCES ${SOURCE_DIR}/*.cpp)

file(GLOB TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/UnitTests/*.cpp)

# List all header files with .h extension
file(GLOB HEADERS ${HEADER_DIR}/*.h)

# Add the external directory for CppUTest
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../ThirdParty/CppUTest ThirdParty/CppUTest)
enable_testing()

# Create a static library for the application code
add_library(squirrel_defender_lib STATIC ${SOURCES} ${HEADERS})

# Locate the static library
# set(STATIC_LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build) # Adjust the path if necessary
find_library(SQUIRREL_DEFENDER_LIB NAMES squirrel_defender_lib PATHS ${STATIC_LIBRARY_DIR})

# Compile the squirreldefender program
if (USE_JETSON)
    cuda_add_executable(squirreldefender ${SOURCES} ${HEADERS})
    target_link_libraries(squirreldefender jetson-inference)
    target_link_libraries(squirreldefender jsoncpp)
    target_link_libraries(squirreldefender ${GST_LIBRARIES} ${GLIB_LIBRARIES} ${GOBJECT_LIBRARIES} ${GST_APP_LIBRARIES})
elseif(USE_WSL)
    add_executable(squirreldefender ${SOURCES} ${HEADERS})
endif()

# Compile the unit_tests program
if (USE_JETSON)
    cuda_add_executable(unit_tests ${TEST_SOURCES} ${HEADERS})
    target_link_libraries(unit_tests jetson-inference)
    target_link_libraries(unit_tests jsoncpp)
    add_test(NAME unit_tests COMMAND unit_tests)
    target_link_libraries(unit_tests squirrel_defender_lib CppUTest CppUTestExt)
    
elseif(USE_WSL)
    add_executable(unit_tests ${TEST_SOURCES} ${HEADERS})
    add_test(NAME unit_tests COMMAND unit_tests)
    target_link_libraries(unit_tests squirrel_defender_lib CppUTest CppUTestExt)

endif()
