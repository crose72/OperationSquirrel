cmake_minimum_required(VERSION 3.10)
project(ButtonPressControl)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find the JetsonGPIO library
find_library(JETSONGPIO_LIB JetsonGPIO REQUIRED)

# Add the executable
add_executable(main main.cpp)

# Link the JetsonGPIO library
target_link_libraries(main ${JETSONGPIO_LIB})

# Include directories for the JetsonGPIO library
include_directories(/usr/local/include/JetsonGPIO)
