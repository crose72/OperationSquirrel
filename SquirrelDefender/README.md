# Description
This folder contains the code for the Squirrel Defender and most of its dependencies.  Some dependencies for the Jetson Nano or other companion computer are found elsewhere, but instructions for how to set those up will be provided in a different file.  This readme will explain how to compile and run this code as well as provide an outline for how the code is structured.  Eventually everything needed will be contained within this folder.

## How to compile and run the program

1. Execute `mkdir build` to create a build directory
2. Execute `cd build` to go to the build folder
3. Execute `cmake ..` to generate build files
4. Execute `make -j$(nproc)` for fast builds
5. Execute `sudo ./squirreldefender` to run the program
6. Execute `./unit_tests` to run the unit tests

## Notes
- Followed the instructions here to update the linker path (it didn't work after only doing this though) https://stackoverflow.com/questions/480764/linux-error-while-loading-shared-libraries-cannot-open-shared-object-file-no-s 
- Copied all .so files from `SquirrelDefender/lib/` to `/usr/local/lib/` on the jetson
- Copied all folders from `SquirrelDefender/inc/` to `/usr/local/include/` on the jetson
- Updated the shared library cache with `sudo ldconfig -v`

## CMakeLists explanation

- Enable either USE_JETSON or USE_WSL by turning them ON or OFF (only one at a time)
    - `option(USE_JETSON "Enable Jetson Nano specific features" ON)`
    - `option(USE_WSL "Enable WSL specific features" OFF)`
- Default release is Debug (enables print statements).  To turn off print statements and other debugging
  features compile a Release type build
    - `cmake -DCMAKE_BUILD_TYPE=Release ..`

#### Other preprocessing directives will be added to configure the code to enable or disable other features

## Code explanation

- `appsrc` - source code files
- `apphdr` - source header files
- `inc` - header files for external libraries
- `lib` - external compiled libraries
- `tests` - header files used to run flight tests, whatever you want the drone to do, hard code it in here
- `UnitTests` - unit test files

#### Further descriptions to come