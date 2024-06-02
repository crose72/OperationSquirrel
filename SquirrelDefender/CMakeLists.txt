# Require CMake 2.8 or greater
cmake_minimum_required(VERSION 2.8)

# Add options to manually enable or disable features
option(USE_JETSON "Enable Jetson Nano specific features" ON)
option(USE_WSL "Enable WSL specific features" OFF)

if (USE_JETSON)
    project(SquirrelDefender)
    option(USE_UART "Use UART functionality for Jetson Nano -> SITL or Jetson Nano -> real drone" ON)
    option(USE_TCP "Use TCP functionality for WSL -> WSL SITL" OFF)
    add_definitions(-DUSE_JETSON)
    add_definitions(-DUSE_UART)
elseif(USE_WSL)
    project(SquirrelDefender C CXX)
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

# Compile the main program
if (USE_JETSON)
    cuda_add_executable(main ${SOURCES} ${HEADERS})
    target_link_libraries(main jetson-inference)
    target_link_libraries(main jsoncpp)
elseif(USE_WSL)
    add_executable(main ${SOURCES} ${HEADERS})
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
