# Description
This folder contains the code for the Squirrel Defender and most of its dependencies.  Some dependencies for the Jetson Nano or other companion computer are found elsewhere, but instructions for how to set those up will be provided in a different file.  This readme will explain how to compile and run this code as well as provide an outline for how the code is structured.  Eventually everything needed will be contained within this folder.

## Before compiling (specifically when linking ORB_SLAM, and pango directly)
- Follow the instructions here to update the linker path https://stackoverflow.com/questions/480764/linux-error-while-loading-shared-libraries-cannot-open-shared-object-file-no-s 
- Copy all .so files from `SquirrelDefender/lib/` to `/usr/local/lib/` on the jetson
- Copy all folders from `SquirrelDefender/inc/` to `/usr/local/include/` on the jetson
- Updated the shared library cache with `sudo ldconfig -v`

## CMakeLists explanation

- Enable either USE_JETSON or USE_WSL by turning them ON or OFF (only one at a time)
    - `option(USE_JETSON "Enable Jetson Nano specific features" ON)`
    - `option(USE_WSL "Enable WSL specific features" OFF)`
- Default release is Debug (enables print statements).  To turn off print statements and other debugging
  features compile a Release type build
    - `cmake -DCMAKE_BUILD_TYPE=Release ..`

#### Other preprocessing directives will be added to configure the code to enable or disable other features

## How to compile and run the program

1. Execute `mkdir build` to create a build directory
2. Execute `cd build` to go to the build folder
3. Execute `cmake ..` to generate build files
4. Execute `make -j$(nproc)` for fast builds or `make` for slow builds
5. Execute `sudo ./squirreldefender` to run the program
6. Execute `./unit_tests` to run the unit tests
7. If the video doesn't display on your monitor, try executing `sudo systemctl restart nvargus-daemon`

## Folder structure

- `appsrc` - source code files
- `apphdr` - source header files
- `inc` - header files for external libraries
- `lib` - external compiled libraries
- `tests` - header files used to run flight tests, whatever behavior you want to hard code the drone to do (especially useful when testing in WSL with virtual jetson)
- `UnitTests` - all the code for unit tests, including helper files
- `params.json` - parameters defined here are adjustable at runtime (not recommended with real vehicle, hardcode all of the parameters when using real vehicle)
- `.editorconfig` - should automatically format the code when saving (if it doesn't then check your settings in vs code)
