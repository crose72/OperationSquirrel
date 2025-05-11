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

# Description

This folder contains the code for the `squirreldefender` program and instructions on how to compile it for different build platforms.  This code assumes use of an IMX219-83 CSI camera on the jetson devices, and the default webcam if compiled on windows.  You can use whatever camera you want on the Jetsons, but if you use a different CSI camera make sure that the video is not updside down.  And if you use a USB camera on the Jetsons then you will need to modify the code slightly.

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
- Nvidia container runtime (if not already installed can follow the instructions at [ISAAC-ROS-Setup.md](https://github.com/crose72/OperationSquirrel/blob/dev/docs/ISAAC-ROS-Setup.md))
- Orin: Jetpack 6.1 (~r36.4 I reckon this should work with Jetpack 6.2, I will try it shortly but I haven't verified it yet)
- B01: Jetpack 4.6.1 (~r32.7.5 is the latest version you can have, container is r32.7.1 based)

### Setup:

Perform the following setup outside of the container and then verify the camera is working in the container.

```
# Create a workspace
mkdir ~/workspaces/os-dev
cd ~/workspaces/os-dev

# Clone the repository
git clone https://github.com/crose72/OperationSquirrel.git --recursive

# Add the following to your ~/.bashrc file
export DISPLAY=:0
export OS_WS=/home/<user>/workspaces/os-dev/

# Restart terminal, or source .bashrc:
source ~/.bashrc

# Enable xserver and display access to the docker container 
# Might need to do before each time running the container
xhost +

# If you get an error, "unable to open dislay :0", try setting to :1 & repeat
# If successfull, you should see something like this:
~/workspaces/os-dev/OperationSquirrel$ xhost +
access control disabled, clients can connect from any host

# If you get an error opening your csi camera, and are running Jetpack 6.2, 
# try following these instructions and enable the CSI camera for
sudo /opt/nvidia/jetson-io/jetson-io.py
Configure Jetson 24pin CSI Connector
Configure for compatible hardware
Camera IMX219 Dual (or whatever CSI camera you're using)
(save, exit, reboot)


# Run the container
./scripts/run_dev_orin.sh

# Test the camera inside the container
nvgstcapture-1.0
```

### Dev container:

Outside of the container edit `SquirrelDefender/CMakeLists.txt` file and enable `BLD_JETSON_ORIN_NANO` or `BLD_JETSON_B01`.

Next, run the script to start up the dev container for your jetson `scripts/run_dev_orin.sh`, `scripts/run_dev_b01.sh`.

A terminal inside the container should open up.  If `SquirrelDefender/build` doesn't exist you need to create it since the dev container opens to that path (`TODO: have dev container create the folder if it doesn't exist?`).  You can now make changes to the code outside of the container, and compile and run the executable inside the container.  Inside the container do the following:

```
# Build
cmake ..
make -j$(nproc)

# Run the executable
./squirreldefender

# If using video playback instead of a camera then execute
./squirreldefender ../test_data/video.mp4
```

### Release container:

If you don't need to change the code and just want to run the precompiled program then you can just do the setup and run `./run_squirreldefender_orin.sh` or `./run_squirreldefender_b01.sh` to run the program for your Jetson.  The purpose of this container is to be deployed onto your drone or other autonomous vehicle since the only thing that it does is run the pre-compiled binary that you created using the dev container from the previous step.

If you want to have the program run as soon as you power on the jetson (for example when your jetson is mounted on your drone and you plug in the battery to your jetson) then you need to add a systemd service and do a couple other things.  First enable xserver and display access for the container by default when you power on the jetson by adding it to your `.xprofile`.  Run these outside of the container:

Create `.xprofile`

```
sudo nano ~/.xprofile

# And then add these lines to it
export DISPLAY=:0
xhost +

# And make it executable
chmod +x ~/.xprofile 
```

Create `squirreldefender.service`

```
# Create a .service file (one is already included in this folder so you can just copy that)
sudo nano /etc/systemd/system/squirreldefender.service

# Add this to the file

    [Unit]fender program
    After=network.target nvargus-daemon.service graphical.target multi-user.target
    Wants=graphical.target

    [Service]
    Environment="OS_WS=/home/<user>/workspaces/os-dev"
    Restart=no
    ExecStartPre=/bin/bash -c 'sleep 5'
    ExecStart=/bin/bash /home/<user>/workspaces/os-dev/OperationSquirrel/scripts/run_squirreldefender_<jetson-type>.sh
    ExecStop=/usr/bin/docker stop squirreldefender
    StandardOutput=journal
    StandardError=journal
    User=root

    [Install]
    WantedBy=multi-user.target  
```

where `<jetson-type>` is `orin` or `b01`.

Enable and start the systemd service you just created

```
sudo systemctl enable squirreldefender.service
sudo systemctl start squirreldefender.service
```

Now your squirreldefender program should start running, and if your jetson is connected to the drone or the simulation on your laptop via FTDI they you'll see some action.  You can also stop, restart, or check the status of the program by using those commands if you need to troubleshoot.  These steps also mean that as soon as you power on your jetson this program will start running unless you disable it or stop it manually.  Log files from this release container are stored in `~/logs` by default.

Some additional useful commands

```
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

### Quickly deploying code changes to the drone:

Let's say you're at the park with your jetson and your drone, and you want to try out different PID gains on the follow algorithm in real life.  Or maybe you want to change the follow distance, or test out some other code changes on the real drone.  How can you make a change to the code, recompile, and update the release container, and then just power the jetson and fly?  Here's how

1. Modify the code using the dev container (follow instructions above)
    - Execute `./scripts/run_dev_<jetson-type>.sh`
    - Make changes to the code
    - Recompile the code
2. Build the SquirrelDefender container
    - Execute `./scripts/build_squirreldefender_<release for your jetson>.sh`
    - (Choose N if you don't need to push the container to docker hub - since you're at the park I don't think you do, or choose Y if you are ready to push it to docker hub)
3. Make sure you've followed the instructions above for the release container to have the container execute when the jetson powers up
    - Stop the squirreldefender service before making changes (instructions above)
    - Start the squirreldefender service whem you're ready to fly again (instructions above)

***Remember to disconnect the uart wires or disconnect the battery from the drone so that the props don't start spinning when you're not ready (if squirreldefender.service is active and started/restarts when you power on the jetson then it will send the command to arm the drone and takeoff so you don't want it near you when that happens)

#### Clock skew error when Jetson no longer has wifi connection

When the Jetson has no internet connection the system time defaults to 1969.  Since you're probably working in the present (2025 and beyond), any files you've touched will likey have a timestamp in the present.  This timestamp is after 1969 by a lot.  So if you go to compile the squirreldefender program the compiler will notice that the files have a timestamp millions of seconds in the future, resulting in a clock skew when recompiling.  To solve this issue, create a systemd service to automatically move the system time to a few seconds after the latest timestamp in the OperationSquirrel repo.

Create a systemd service
```
sudo nano /etc/systemd/system/clock-skew-fix.service
```

```
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

Setup swap.  Either run [setup-swap.sh](https://github.com/crose72/OperationSquirrel/blob/dev/scripts/setup-swap.sh) or you can manually install some swap:

```
sudo fallocate -l 8G /mnt/8GB.swap
sudo mkswap /mnt/8GB.swap
sudo swapon /mnt/8GB.swap
        
Then add the following line to the end of /etc/fstab to make the change persistent:
    /mnt/8GB.swap  none  swap  sw 0  0
```

Copy the script [Install-Jetson-Orin-Nano-dependencies.sh](https://github.com/crose72/OperationSquirrel/blob/dev/scripts/Install-Jetson-Orin-Nano-dependencies.sh) to your favorite GitHub folder because it will clone some repos wherever you run this script, so just make sure to put it wherever you are okay with having those repositories.

### Compile and run the program:

```
# Clone the repository
git clone https://github.com/crose72/OperationSquirrel.git --recursive
cd OperationSquirrel/SquirrelDefender
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

If you want to update your cmake version then you can run the script I made to install 3.28 [Install-CMake-3.28.sh](https://github.com/crose72/OperationSquirrel/blob/dev/scripts/Install-CMake-3.28.sh).

Setup swap.  Either run [setup-swap.sh](https://github.com/crose72/OperationSquirrel/blob/dev/scripts/setup-swap.sh) or you can manually install some swap:

```
sudo fallocate -l 8G /mnt/8GB.swap
sudo mkswap /mnt/8GB.swap
sudo swapon /mnt/8GB.swap
        
Then add the following line to the end of /etc/fstab to make the change persistent:
    /mnt/8GB.swap  none  swap  sw 0  0
```

Copy the script [Install-Jetson-Nano-B01-dependencies.sh](https://github.com/crose72/OperationSquirrel/blob/dev/scripts/Install-Jetson-Nano-B01-dependencies.sh) to your favorite GitHub folder because it will clone some repos wherever you run this script, so just make sure to put it wherever you are okay with having those repositories.

Edit `SquirrelDefender/CMakeLists.txt` file and enable `BLD_JETSON_B01`.

### Compile and run the program:

```
# Clone the repository
git clone https://github.com/crose72/OperationSquirrel.git --recursive
cd OperationSquirrel/SquirrelDefender
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

```
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
    WorkingDirectory=<path-to-build-folder>//SquirrelDefender/build
    StandardOutput=journal
    StandardError=journal
    Restart=always
    User=root

    [Install]
    WantedBy=multi-user.target    
```

Next, enable and start the systemd service you just created

```
sudo systemctl enable squirreldefender.service
sudo systemctl start squirreldefender.service
```

Now your squirreldefender program should start running, and if your jetson is connected to the drone or the simulation on your laptop via FTDI they you'll see some action.  You can also stop, restart, or check the status of the program by using those commands if you need to troubleshoot.  These steps also mean that as soon as you power on your jetson this program will start running unless you disable it or stop it manually.  Log files from this release container are stored in `~/logs` by default.

Some additional useful commands

```
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

The main purpose of this program is to be run on the edge.  It can also be run on a laptop or desktop if you do not have a jetson edge device like the nano or the orin nano.  I won't go into great detail about these build platforms but I will tell you what's required.  If you have more questions feel free to reach out on the discord or try it on your own.  It's basically the same as above, but I use visual studio for my windows project

### Prerequisites:

- CMake >= 3.28 (same rules as above apply for different versions, you have to troubleshoot)
- OpenCV 4.10.0 with Cuda and cuDNN
- Cuda 12.6
- cuDNN 8.9.7.29 (Can try a different version of cuDNN 8, and maybe cuDNN 9, but don't use 9.5.1.17, we tried, didn't work - known issue)
- Json (if you want to use the params file, otherwise optional)

### Setup:

Follow the instructions to install OpenCV 4.10.0 with Cuda 12.6 and cuDNN 8.9 from this medium article I found https://medium.com/@jinscott/build-opencv-on-windows-with-cuda-f880270eadb0.  ***DO NOT COMPILE FOR STATIC LIBS***. You can, it's just that you'd then have to manually link every library file.  I compiled with dynamic libs.  But feel free to do it however is best for you.

Install TensorRT 10.4 following the instructions on their website.

Install CMake and the gui on your computer.

Configure the CMakeList.txt file for your windows build.

Open CMake gui and setup the project using the cmake file in `SquirrelDefender`.

In Visual Studio choose a release type build (You can try debug but I have had issues with that one).

Build the executable.

Before you execute the program make sure the ArduPilot SITL is running.  The windows build depends on that before it can continue to execute.

## Linux laptop or desktop

I'm going to leave you to figure out how to compile it on your linux machine based on the instructions above.  It's pretty much the same thing.  Here is something to help you get started on installing opencv with cuda and cudnn on linux though https://medium.com/@juancrrn/installing-opencv-4-with-cuda-in-ubuntu-20-04-fde6d6a0a367.
