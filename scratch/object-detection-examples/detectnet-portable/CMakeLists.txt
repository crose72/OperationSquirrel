# require CMake 2.8 or greater
cmake_minimum_required(VERSION 2.8)

# Include the "c" folder and its subfolders
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/c)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/c/calibration)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/c/experimental)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/c/plugins)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/c/tracking)

# declare detectnet project
project(detectnet)

# import jetson-inference and jetson-utils packages
find_package(jetson-utils)
find_package(jetson-inference)

# find VPI package (optional)
find_package(VPI 2.0)

# CUDA is required
find_package(CUDA)

# add directory for libnvbuf-utils to program
link_directories(/usr/lib/aarch64-linux-gnu/tegra)

# List your source files
set(SOURCES
    main.cpp
    camera_handler.cpp
	target_tracking.cpp
)

# List your header files
set(HEADERS
    camera_handler.h  # Add your header files here
    # Other header files...
)

# compile the detectnet program
cuda_add_executable(detectnet ${SOURCES} ${HEADERS})

# link detectnet to jetson-inference library
target_link_libraries(detectnet jetson-inference)

