-*-*+*# Description
This folder contains the code for the Squirrel Defender and most of its dependencies.  Some dependencies for the Jetson Nano or other companion computer are found elsewhere, but instructions for how to set those up will be provided in a different file.  This readme will explain how to compile and run this code as well as provide an outline for how the code is structured.

## How to compile and run the program
1. Execute `make` to build the example
2. Execute `sudo ./main` to run the program
3. Execute `make clean` to remove the build files including the executable

## Makefile explanation
- Uncomment one of the following lines in the Makefile to compile for use on
   the Jetson Nano using UART or on WSL2 using a TCP connection depending on where this
   code is being executed.  Only one of them should be used at a time
   - CFLAGS += -DUSE_UA-RT
   - CFLAGS += -DUSE_TCP

#### Other preprocessing directives will be added to configure the code to enable or disable other features

## Code explanation
- `appsrc` contains all source code files
- `apphdr` contains all header files
- `lib` contains all libraries used 

#### Further descriptions to come