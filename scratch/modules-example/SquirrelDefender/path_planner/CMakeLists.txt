cmake_minimum_required(VERSION 3.10)

# Project name
project(path_planner VERSION 1.0.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build options
option(PATH_PLANNER_ENABLE_DEBUG "Enable debug logging" ON)
option(PATH_PLANNER_ENABLE_TESTS "Build unit tests" ON)
option(PATH_PLANNER_ENABLE_EXAMPLES "Build examples" ON)

# Set include directories
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(
    ${INCLUDE_DIR}
)

# Source files
set(SOURCES
    src/path_planner/path_planner.cpp
    src/path_planner/path_planner_delivery.cpp
    src/path_planner/path_planner_factory.cpp
    src/path_planner/path_planner_follower.cpp
    src/path_planner/path_planner_grid.cpp
)

# Header files
set(HEADERS
    include/path_planner/path_planner.h
    include/path_planner/path_planner_delivery.h
    include/path_planner/path_planner_follower.h
    include/path_planner/path_planner_grid.h
    include/path_planner/path_planner_types.h
)

# Create library
add_library(path_planner STATIC ${SOURCES} ${HEADERS})

# Set include directories for the target
target_include_directories(path_planner
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Link dependencies
target_link_libraries(path_planner
    PRIVATE
)

# Set compile definitions
target_compile_definitions(path_planner
    PRIVATE
    $<$<BOOL:${PLANNER_ENABLE_DEBUG}>:PATH_PLANNER_DEBUG>
)

# Testing
if(PATH_PLANNER_ENABLE_TESTS)
    enable_testing()
    
    # Add test executable
    add_executable(path_planner_test
        tests/test_path_planner.cpp
    )
    
    # Link test executable with planner library
    target_link_libraries(path_planner_test
        PRIVATE
        path_planner
    )
    
    # Add test
    add_test(NAME path_planner_test COMMAND path_planner_test)
    
    # Set debug configuration for test
    set_target_properties(path_planner_test PROPERTIES
        VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    )
endif()

# Examples
if(PATH_PLANNER_ENABLE_EXAMPLES)
    # Add example executables
    add_executable(path_planner_example
        examples/src/path_planner_example.cpp
    )
    
    # Link example executables with planner library
    target_link_libraries(path_planner_example
        PRIVATE
        path_planner
    )
    
    # Set debug configuration for examples
    set_target_properties(path_planner_example PROPERTIES
        VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    )
    
endif()

# Install rules
install(TARGETS path_planner
    EXPORT path_planner-targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/path_planner
    DESTINATION include
)

# Export targets
install(EXPORT path_planner-targets
    FILE path_planner-targets.cmake
    NAMESPACE path_planner::
    DESTINATION lib/cmake/path_planner
)

# Create and install package configuration file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/path_planner-config-version.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/path_planner-config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/path_planner-config.cmake"
    INSTALL_DESTINATION lib/cmake/path_planner
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/path_planner-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/path_planner-config-version.cmake"
    DESTINATION lib/cmake/path_planner
) 