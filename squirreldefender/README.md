
# Overview

This folder contains the source code for SquirrelDefender, the autonomous perception + control stack used in Operation Squirrel.

It includes real-time:

- Object detection (YOLOv8 TensorRT CUDA)
- Tracking
- Target following & control logic
- Mission behaviors
- MCAP logging

Supported Jetson platforms:

- Orin Nano
- Orin Nx
- AGX Orin
- Nano B01

## Quick Start (Jetson)

On the Jetson, perform the following setup outside of the container and then verify the camera is working in the container.

1. Setup workspace and environment

    ```bash
    # Create a workspace
    mkdir ~/workspaces/os-dev
    cd ~/workspaces/os-dev

    # Clone the repository
    git clone https://github.com/crose72/operationsquirrel.git --recursive

    # Run setup script (installs env + CSI camera config)
    cd operationsquirrel/scripts
    ./setup.sh squirreldefender --jetson=orin # or --jetson=b01
    ```

2. Finish setup

    ```bash
    source ~/.bashrc
    chmod +x ~/.xprofile
    sudo reboot now
    ```

3. After reboot start the dev container
    ```bash
    # Run the container
    cd ~/workspaces/os-dev/operationsquirrel/scripts
    ./run.sh dev orin # or dev b01

    # Test the camera inside the container (sanity check)
    nvgstcapture-1.0
    ```

4. Configure the build

    Edit:
    
    ```bash
    squirreldefender/CMakeLists.txt
    ```
    
    and enable ONLY ONE of:

    ```bash
    BLD_JETSON_ORIN=ON
    BLD_JETSON_B01=ON
    ```

5. Build inside the dev container

    ```bash
    cd build    # container opens here automatically

    cmake -S .. -B .cmake
    cmake --build .cmake -- -j$(nproc)

    # Run the executable
    ./squirreldefender

    # Or run using a video file
    ./squirreldefender ../test_data/video.mp4

    ```

## Folder structure

```
squirreldefender/
├── appsrc/            # Primary C++ source files
├── apphdr/            # Header files
├── modules/           # External components (trtinfer, kf, mcap, etc.)
├── libraries/         # Standalone shared code (logger, utilities, etc.)
├── proto/             # Protobuf definitions for MCAP logging
├── cmake/             # CMake helper modules & configuration files
├── params.json        # Control + system parameters
├── test_suite/        # Test harness entry points & hardcoded test flights
```