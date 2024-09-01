cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(main)

add_executable(main
    main.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(main
    MAVSDK::mavsdk
    ${CMAKE_CXX_STANDARD_LIBRARIES}
)


if(NOT MSVC)
    add_compile_options(main PRIVATE -Wall -Wextra)
else()
    add_compile_options(main PRIVATE -WX -W2)
endif()

if(MINGW OR MSVC)
    # For M_PI
    add_target_definitions(main "-D_USE_MATH_DEFINES")
endif()