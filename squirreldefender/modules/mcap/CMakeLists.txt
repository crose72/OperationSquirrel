cmake_minimum_required(VERSION 3.22)

# Header-only MCAP library
add_library(mcap INTERFACE)

# Expose include directory
target_include_directories(mcap INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# MCAP requires zlib, lz4, zstd, threads
find_package(ZLIB REQUIRED)
find_package(Threads REQUIRED)

target_link_libraries(mcap INTERFACE
    ZLIB::ZLIB
    Threads::Threads
    ${LZ4_LIBRARY}
    ${ZSTD_LIBRARY}
)
