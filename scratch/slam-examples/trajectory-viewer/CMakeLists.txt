cmake_minimum_required(VERSION 3.1)

project(ORB_SLAM3_Pangolin_Visualization)

# Find Pangolin
find_package(Pangolin REQUIRED)

# Include directories
include_directories(${Pangolin_INCLUDE_DIRS})

# Add your ORB_SLAM3 source files here
set(ORB_SLAM3_SRC_FILES
        # Add your ORB_SLAM3 source files here
        )

# Add executable
add_executable(main main.cpp ${ORB_SLAM3_SRC_FILES})

# Link libraries
target_link_libraries(main ${Pangolin_LIBRARIES})

# Link against Pangolin libraries in the "lib" folder
target_link_libraries(main
                      libpango_core.so
                      # Add other Pangolin dependencies here
                      )

# If other Pangolin dependencies are needed, add them here
# target_link_libraries(main other_dependency_library)

