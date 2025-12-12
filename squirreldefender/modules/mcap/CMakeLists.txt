cmake_minimum_required(VERSION 3.22.1)

project(mcap LANGUAGES CXX VERSION 1.4.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ======================================
# Paths
# ======================================
set(MCAP_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")

# ======================================
# Sources
# ======================================
# Create a dummy mcap.cpp file to create a static library
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/mcap.cpp"
"#define MCAP_IMPLEMENTATION\n#include <mcap/mcap.hpp>\n")
set(MCAP_SOURCES "${CMAKE_CURRENT_BINARY_DIR}/mcap.cpp")

# ======================================
# Library target
# ======================================
add_library(mcap STATIC ${MCAP_SOURCES})

target_include_directories(mcap PUBLIC
    $<BUILD_INTERFACE:${MCAP_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# ======================================
# Optional compression dependencies
# ======================================
set(MCAP_DEPENDENCIES "")

find_package(lz4 QUIET)
if (lz4_FOUND)
    message(STATUS "Found LZ4 at ${lz4_DIR}")
    list(APPEND MCAP_DEPENDENCIES LZ4::lz4)
else()
    message(WARNING "LZ4 not found; disabling LZ4 support in MCAP")
    target_compile_definitions(mcap PUBLIC MCAP_COMPRESSION_NO_LZ4)
endif()

find_package(zstd QUIET)
if (zstd_FOUND)
    message(STATUS "Found ZSTD at ${zstd_DIR}")
    list(APPEND MCAP_DEPENDENCIES zstd::libzstd)
else()
    message(WARNING "ZSTD not found; disabling ZSTD support in MCAP")
    target_compile_definitions(mcap PUBLIC MCAP_COMPRESSION_NO_ZSTD)
endif()

target_link_libraries(mcap PUBLIC ${MCAP_DEPENDENCIES})

# ======================================
# Installation: headers + library + config
# ======================================
include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    "${PROJECT_BINARY_DIR}/mcap-config-version.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(TARGETS mcap
    EXPORT mcap-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY ${MCAP_INCLUDE_DIR} DESTINATION include)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/mcap-config.cmake.in"
    "${PROJECT_BINARY_DIR}/mcap-config.cmake"
    INSTALL_DESTINATION lib/cmake/mcap
)

install(EXPORT mcap-targets DESTINATION lib/cmake/mcap)
install(FILES
    "${PROJECT_BINARY_DIR}/mcap-config.cmake"
    "${PROJECT_BINARY_DIR}/mcap-config-version.cmake"
    DESTINATION lib/cmake/mcap
)
