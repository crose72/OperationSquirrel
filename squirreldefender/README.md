
# Overview

This folder contains the code for the `squirreldefender` program and instructions on how to compile it for different build platforms.  This code assumes use of an IMX219-83 CSI camera on the jetson devices, and the default webcam if compiled on windows.  You can use whatever camera you want on the Jetsons, but if you use a different CSI camera make sure that the video is not updside down.  And if you use a USB camera on the Jetsons then you will need to modify the code slightly.

# Table of Contents

- [Description](#description)
- [Folder Structure](#folder-structure)
- [Building with Docker](#building-with-docker)
  - [Jetson Orin Nano and Jetson Nano B01](#jetson-orin-nano-and-jetson-nano-b01)
    - [Prerequisites](#prerequisites)
    - [Setup](#setup)
    - [Dev Container](#dev-container)
    - [Release Container](#release-container)
    - [Quickly Deploying Code Changes to the Drone](#quickly-deploying-code-changes-to-the-drone)
      - [Clock Skew Error When Jetson No Longer Has WiFi Connection](#clock-skew-error-when-jetson-no-longer-has-wifi-connection)
- [Building Without Docker](#building-without-docker)
  - [Jetson Orin Nano](#jetson-orin-nano)
    - [Prerequisites](#prerequisites-1)
    - [Setup](#setup-1)
    - [Compile and Run the Program](#compile-and-run-the-program)
  - [Jetson Nano B01](#jetson-nano-b01)
    - [Prerequisites](#prerequisites-2)
    - [Setup](#setup-2)
    - [Compile and Run the Program](#compile-and-run-the-program-1)
  - [Windows](#windows)
    - [Prerequisites](#prerequisites-3)
    - [Setup](#setup-3)
  - [Linux Laptop or Desktop](#linux-laptop-or-desktop)

# Folder structure

- `appsrc` - source code files
- `apphdr` - source header files
- `modules` - standalone code that is used in the main program, but is not the main functionality
- `test_flights` - header files used to run flight tests, whatever behavior you want to hard code the drone to do (especially useful when testing in WSL with virtual jetson)
- `test_data` - csv files used to pass as inputs to the program using the test harness
- `test_suite` - custom main.cpp files for testing specific segments of the code
- `params.json` - control parameters
- `.editorconfig` - should automatically format the code when saving (if it doesn't then check your settings in vs code)

# Building with docker

## Jetson Orin Nano and Jetson Nano B01

### Prerequisites:

- Docker
- Nvidia container runtime
- Orin: Jetpack 6.1 or 6.2 (container is r36.4 based)
- B01: Jetpack 4.6.1 (container is r32.7.1 based)

### Setup:

Perform the following setup outside of the container and then verify the camera is working in the container.

```bash
# Create a workspace
mkdir ~/workspaces/os-dev
cd ~/workspaces/os-dev

# Clone the repository
git clone https://github.com/crose72/operationsquirrel.git --recursive

# Add the user to docker
sudo usermod -aG docker $USER && newgrp docker

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

### Dev container:

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

A terminal inside the container should open up.  If `squirreldefender/build` doesn't exist on your jetson then you need to create it since the dev container opens to that path (the `setup.sh` should take care of this).  You can now make changes to the code outside of the container (the code is mounted into the container), and compile and run the executable inside the container.  Inside the container do the following:

```bash
# Build
cmake ..
make -j$(nproc)

# Run the executable
./squirreldefender

# If using video playback instead of a camera then execute
./squirreldefender ../test_data/video.mp4
```

### Release container:

If you don't need to change the code and just want to run the precompiled program then you can just do the setup and run `./run.sh squirreldefender orin` or `./run.sh squirreldefender b01` to run the program for your Jetson.  The purpose of this container is to be deployed onto your drone or other autonomous vehicle since the only thing that it does is run the pre-compiled binary that you created using the dev container from the previous step.

If you want to have the program run as soon as you power on the jetson (for example when your jetson is mounted on your drone and you plug in the battery to your jetson) then you need to do the setup, and then enable the `squirreldefender.service` by doing

```bash
sudo systemctl enable squirreldefender.service
```

Now your squirreldefender program should start running every time your jetson is powered on (until you disable the service), and if your jetson is connected to the drone or the simulation on your laptop via FTDI they you'll see some action.  You can also stop, restart, or check the status of the program by using those commands if you need to troubleshoot.  These steps also mean that as soon as you power on your jetson this program will start running unless you disable it or stop it manually.  Log files from this release container are stored in `~/logs` by default.

Some additional useful commands

```bash
# Save the file and reload the daemon
sudo systemctl daemon-reload

# Enable the service on startup
sudo systemctl enable squirreldefender.service

# Disable the service on startup
sudo systemctl disable squirreldefender.service

# Start the service
sudo systemctl start squirreldefender.service

# Stop the service
sudo systemctl stop squirreldefender.service

# Restart the service
sudo systemctl restart squirreldefender.service

# Check status of the service
sudo systemctl status squirreldefender.service
```

### Quickly deploying code changes to the drone:

Let's say you're at the park with your jetson and your drone, and you want to try out different PID gains on the follow algorithm in real life.  Or maybe you want to change the follow distance, or test out some other code changes on the real drone.  How can you make a change to the code, recompile, and update the release container, and then just power the jetson and fly?

***Remember to disconnect the uart wires or disconnect the battery from the drone so that the props don't start spinning when you're not ready (if squirreldefender.service is active and started/restarts when you power on the jetson then it will send the command to arm the drone and takeoff so you don't want it near you when that happens)

1. Stop squirreldefender service
    - `sudo systemctl stop squirreldefender.service`
2. Modify the code using the dev container (follow instructions above)
    - Execute `./run.sh dev orin` (or whatever jetson you're using)
    - Make changes to the code
    - Recompile the code
3. Build the squirreldefender container
    - Execute `./build.sh squirreldefender orin` (or whatever jetson you're using)
    - (Choose N if you don't need to push the container to docker hub - since you're at the park I don't think you do)
4. Enable the squirreldefender service (if not already enabled)
    - `sudo systemctl enable squirreldefender.service`


#### Clock skew error when Jetson no longer has wifi connection

When the Jetson has no internet connection the system time defaults to 1969.  Since you're probably working in the present (2025 and beyond), any files you've touched will likey have a timestamp in the present.  This timestamp is after 1969 by a lot.  So if you go to compile the squirreldefender program the compiler will notice that the files have a timestamp millions of seconds in the future, resulting in a clock skew when recompiling.  To solve this issue, create a systemd service to automatically move the system time to a few seconds after the latest timestamp in the operationsquirrel repo.

The `setup.sh` should take care of this, but here is the manual setup for your benefit.

Create a systemd service

```bash
sudo nano /etc/systemd/system/clock-skew-fix.service
```

```bash
# Add this to the file

    [Unit]
    Description=Ensure system clock is ahead of all file timestamps
    After=multi-user.target
    [Service]
    Type=oneshot
    ExecStart=/bin/bash -c '
      # Replace with your actual project/code path
      latest=$(find /home/user/project -type f -exec stat -c %%Y {} + 2>/dev/null | sort -n | tail -1)
      now=$(date +%%s)
      if [ -n "$latest" ] && [ "$now" -lt "$latest" ]; then
        echo "⏩ Clock behind file timestamps, setting time forward..."
        date -s "@$((latest + 10))"
      fi
    '
    [Install]
    WantedBy=multi-user.target
```

Next, enable and start the systemd service you just created

```
sudo systemctl enable clock-skew-fix.service
sudo systemctl start clock-skew-fix.service
```

# Building without docker

## Jetson Orin Nano

There are two containers for development on the Jetson Orin Nano.  A dev container, used for compiling and testing the code, and a release container, used for just running the compiled executable.  All of the dependencies are inside the container so you just have to run them :)

### Prerequisites:

- Jetpack 6.1 (~r36.4 accidentally discovered that it also works with Jetpack 6.2)
- CMake >= 3.28 (same rules as above apply for different versions, you have to troubleshoot)
- OpenCV 4.10.0 with Cuda and cuDNN
- Cuda 12.6 (should be default on the orin)
- cuDNN 9.3.0 (default on )
- Json (if you want to use the params file, otherwise optional)
- Amardillo
- LAPACK
- BLAS
- TensorRT >= 10

Note: Defaults on the Orin are fine for all of these if you have JP6.1 or 6.2.  I would recommend installing the correct opencv version using the script provided, however.

### Setup:

Setup swap.  Either run [setup-swap.sh](https://github.com/crose72/operationsquirrel/blob/dev/scripts/setup-swap.sh) or you can manually install some swap:

```
sudo fallocate -l 8G /mnt/8GB.swap
sudo mkswap /mnt/8GB.swap
sudo swapon /mnt/8GB.swap
        
Then add the following line to the end of /etc/fstab to make the change persistent:
    /mnt/8GB.swap  none  swap  sw 0  0
```

Copy the script [Install-Jetson-Orin-Nano-dependencies.sh](https://github.com/crose72/operationsquirrel/blob/dev/scripts/Install-Jetson-Orin-Nano-dependencies.sh) to your favorite GitHub folder because it will clone some repos wherever you run this script, so just make sure to put it wherever you are okay with having those repositories.

### Compile and run the program:

```bash
# Clone the repository
git clone https://github.com/crose72/operationsquirrel.git --recursive
cd operationsquirrel/squirreldefender
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)

# Run the executable
sudo ./squirreldefender

# If using video playback instead of a camera then execute
sudo ./squirreldefender ../test_data/video.mp4
```

## Jetson Nano B01

### Prerequisites:

- OpenCV 4.10.0 with Cuda and cuDNN
- Jetson Inference
- Json
- CMake >= 3.28 (Earlier versions will likely work but you will have to troubleshoot that.  Probably don't use lower than version 3)
- Jetpack 4.6 (r32.7.5, though it should work with some slightly earlier versions)
- Amardillo
- LAPACK
- BLAS

### Setup:

If you want to update your cmake version then you can run the script I made to install 3.28 [Install-CMake-3.28.sh](https://github.com/crose72/operationsquirrel/blob/dev/scripts/Install-CMake-3.28.sh).

Setup swap.  Either run [setup-swap.sh](https://github.com/crose72/operationsquirrel/blob/dev/scripts/setup-swap.sh) or you can manually install some swap:

```bash
sudo fallocate -l 8G /mnt/8GB.swap
sudo mkswap /mnt/8GB.swap
sudo swapon /mnt/8GB.swap
        
Then add the following line to the end of /etc/fstab to make the change persistent:
    /mnt/8GB.swap  none  swap  sw 0  0
```

Copy the script [Install-Jetson-Nano-B01-dependencies.sh](https://github.com/crose72/operationsquirrel/blob/dev/scripts/Install-Jetson-Nano-B01-dependencies.sh) to your favorite GitHub folder because it will clone some repos wherever you run this script, so just make sure to put it wherever you are okay with having those repositories.

Edit `squirreldefender/CMakeLists.txt` file and enable `BLD_JETSON_B01`.

### Compile and run the program:

```bash
# Clone the repository
git clone https://github.com/crose72/operationsquirrel.git --recursive
cd operationsquirrel/squirreldefender
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)

# Run the executable
sudo ./squirreldefender

# If using video playback instead of a camera then execute
sudo ./squirreldefender ../test_data/video.mp4
```

If you run into issues with the camera try `sudo systemctl restart nvargus-daemon`.

If you want to have the program run as soon as you power on the jetson (for example when your jetson is mounted on your drone) then you need to add a systemd service and do a couple other things.

```bash
# Create a .service file (one is already included in this folder so you can just copy that)
sudo nano /etc/systemd/system/squirreldefender.service

# Add this to the file

    [Unit]
    Description=Squirrel Defender program
    After=network.target nvargus-daemon.service

    [Service]
    Type=simple
    ExecStartPre=/bin/systemctl restart nvargus-daemon.service
    ExecStart=<path-to-exe>/squirreldefender
    WorkingDirectory=<path-to-build-folder>//squirreldefender/build
    StandardOutput=journal
    StandardError=journal
    Restart=always
    User=root

    [Install]
    WantedBy=multi-user.target    
```

Next, enable and start the systemd service you just created

```bash
sudo systemctl enable squirreldefender.service
sudo systemctl start squirreldefender.service
```

Now your squirreldefender program should start running, and if your jetson is connected to the drone or the simulation on your laptop via FTDI they you'll see some action.  You can also stop, restart, or check the status of the program by using those commands if you need to troubleshoot.  These steps also mean that as soon as you power on your jetson this program will start running unless you disable it or stop it manually.  Log files from this release container are stored in `~/logs` by default.

Some additional useful commands

```bash
# Some useful commands

# Save the file and reload the daemon
sudo systemctl daemon-reload

# Enable the service
sudo systemctl enable squirreldefender.service

# Start the service
sudo systemctl start squirreldefender.service

# Restart the service
sudo systemctl restart squirreldefender.service

# Check status of the service
sudo systemctl status squirreldefender.service
```

## Windows

The main purpose of this program is to be run on the edge.  It can also be run on a laptop or desktop if you do not have a jetson edge device like the nano or the orin nano.

### Prerequisites:

- Docker Desktop
- WSL2 with Ubuntu-22.04

### Setup:

This container was built with CUDA enabled OpenCV for a GPU with compute capability of 8.6.  If your GPU has a different capability then a new container will have to be built to accomodate that compute capability.  If you want to do that reach out in the discord and I can help with that.

In WSL2 (or on your linux machine) perform the following setup steps:

```bash
# Create a workspace
mkdir ~/workspaces/os-dev
cd ~/workspaces/os-dev

# Clone the repository
git clone https://github.com/crose72/operationsquirrel.git --recursive

# Add the user to docker
sudo usermod -aG docker $USER && newgrp docker

# Add these lines to the end of your bashrc file
export DISPLAY=:0
export OS_WS=/home/crose72/workspaces/os-dev/

sudo nano ~/.bashrc # or you can use vim if you prefer

# Source .bashrc file (reset)
source ~/.bashrc

# Run the container
cd ~/workspaces/os-dev/operationsquirrel/scripts
./scripts/run_dev_ubuntu-22.04.sh
```

Edit the `mav_serial.cpp` file IP address argument to ensure that the program can communicate with ArduPilot SITL.  In CMakeLists.txt set `BLD_WSL` to `ON`.  Inside the container you can now build and run the executable:

```bash
cmake ..
make -j$(nproc)
./squirreldefender <path-to-video-if-no-camera-attached>
```

Before you execute the program make sure the ArduPilot SITL is running.  The windows build depends on that before it can continue to execute.

## Linux laptop or desktop

Same steps as the windows instructions above.
