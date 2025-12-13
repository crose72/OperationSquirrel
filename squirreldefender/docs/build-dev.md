# Building squirreldefender on the Jetson

This document is a more detailed overview of the instructions in the `Quick Start` section in `squirreldefender/README.md`

## Setup Workspace

### Prerequisites:

- Orin: Jetpack 6.1 or 6.2 (container is r36.4 based)
- B01: Jetpack 4.6.1 (container is r32.7.1 based)

### Setup:

On the Jetson, perform the following setup outside of the container and then verify the camera is working in the container.

```bash
# Create a workspace
mkdir ~/workspaces/os-dev
cd ~/workspaces/os-dev

# Clone the repository
git clone https://github.com/crose72/operationsquirrel.git --recursive

# Go to and execute the setup script
cd operationsquirrel/scripts
../setup.sh squirreldefender --jetson=orin # for the Orin nano, --jetson=b01 for the B01 nano

# This setup script will execute a python script that will allow you to configure your csi camera.
# It will prompt you to reboot the jetson for the changes to take effect.  Choose to exit without
# rebooting.  If you choose to reboot, make sure you follow the next steps to source the bashrc 
# file and make the xprofile executable.

# Source .bashrc file (reset)
source ~/.bashrc

# Make .xprofile executable
chmod +x ~/.xprofile

# Reboot the jetson (if you didn't accidentally reboot when configuring the camera)
sudo reboot now

# Run the container
cd ~/workspaces/os-dev/operationsquirrel/scripts
./run.sh dev orin

# Test the camera inside the container (sanity check before testing squirreldefender)
nvgstcapture-1.0
```

## Building squirreldefender

### Dev container

Outside of the container edit `squirreldefender/CMakeLists.txt` file and enable `BLD_JETSON_ORIN` or `BLD_JETSON_B01`.

Next, run the script to start up the dev container for your jetson 

```bash
# Go to scripts folder
cd operationsquirrel/scripts

# Run the container
./run.sh dev orin

# or for the B01 nano
./run.sh dev b01
```

A terminal inside the container should open up.  The container opens up to `squirreldefender/build` (the `setup.sh` should take care of this).  You can now make changes to the code outside of the container (the code is mounted into the container), and compile and run the executable inside the container.  Inside the container do the following:

```bash
# Build
cd build # if not already in the build folder
cmake -S .. -B .cmake
cmake --build .cmake -- -j$(nproc)

# Run the executable
./squirreldefender

# If using video playback instead of a camera then execute
./squirreldefender ../test_data/video.mp4
```