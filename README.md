# OperationSquirrel

## The drone
<img src="https://github.com/user-attachments/assets/e551dd46-b7c2-478f-9a46-858f54ebddc5" alt="Image 1" width="640">

### Drone stabilization test

https://github.com/user-attachments/assets/681405a0-4fe6-4277-aa67-8812df1160f3

### Autonomous drone following in simulation (hardware in loop)

https://github.com/user-attachments/assets/a6b1fb76-5357-4e33-b026-c60e2d991b9d

### Autonomous drone following in real life

https://github.com/user-attachments/assets/12bda7be-26f7-4e50-8a76-f6e40bb1b42d

## Description

This project is aimed at creating an autonomous drone to deliver static and/or dynamic payloads to a moving target.  Advanced control methods, path planning algorithms, localization, AI/ML techniques and more will be deployed to achieve this goal.  The engineering feats accomplished along the way are stepping stones for other exciting problems.  The work done for this project can be expanded to protect crops from birds, home security, deliveries, or any other creative use.

- `arduPilot` - parameters, firmware and any other ardupilot and vehicle specific things
- `docs` - instructions on how to set up the workflow and various other things
- `jetson` - specific libraries, headers, and other files already organized for you :
- `scratch` - miscellaneous experimental code, a sandbox to try out code and save it to the repo
- `scripts` - scripts to automatically install dependencies or do other tasks
- `SquirrelDefender` - the main application code
- `ThirdParty` - third party software included as submodules
- `tools` - tools specific to operation squirrel for processing data or performing other useful tasks
- `wsl` - my recompiled WSL2 kernel that supports USBIPD (connecting usb devices to wsl) so if you follow the instructions in that folder you'll have that capability right away

Developer chat: <https://discord.gg/Uxg9tpVMP9>

#### If you need help or the instructions are unclear, please reach out in the discord.  It's only me maintaining this repo and I only have so much time to edit the documentation if no one is looking at it.  I'm more focused on the fun stuff like make drones fly by themselves and delivering payloads! :)

## Getting started

1. Compiling the `squirreldefender` program
    - If you have the Jetson Nano B01
        1. Follow the instructions in [Install-dependencies](https://github.com/crose72/OperationSquirrel/blob/dev/scripts/Install-dependencies.md) to setup swap and install the dependencies.
        2. Folow the instructions in [Building-SquirrelDefender](https://github.com/crose72/OperationSquirrel/blob/dev/SquirrelDefender/Building-SquirrelDefender.md) to compile the code on the Jetson.
    - If you have the Jetson Orin Nano
        1. Good news, almost nothing is required because I built a container for you :)
        2. Install docker and the nvidia container runtime.  Follow the isaac ros setup instructions at https://nvidia-isaac-ros.github.io/getting_started/index.html.  Or you can use the instructions I made after I followed the isaac ros steps (it can be confusing), and you can find them here [ISAAC-ROS-Setup](https://github.com/crose72/OperationSquirrel/blob/dev/docs/ISAAC-ROS-Setup.md)
        3. Create a workspace and add it to the 
            ```
            mkdir ~/workspaces/os-dev/

            echo "export OS_WS=/home/<user-name>/workspaces/os-dev/" >> ~/.bashrc
            source ~/.bashrc
            ```
        4. Clone the repository
            ```
            cd ${OS_WS}/
            git clone https://github.com/crose72/OperationSquirrel.git --recursive
            ```
        5. Give video access to the container
            ```
            # Need this everytime
            export DISPLAY=:0 # or add this to your /.bashrc file and then source ~/.bashrc
            xhost +

            # If you don't want to do this everytime then create the following
            sudo nano ~/.xprofile
                                                                                            # And then add these lines to it
            export DISPLAY=:0
            xhost +

            # And make it executable
            chmod +x ~/.xprofile 
            ```
        6. Run the container
            ```
            cd ${OS_WS}/ && \
            ./OperationSquirrel/scripts/run_dev.sh
            ```
        7. The OperationSquirrel repo is now mounted in the container you've just started and you can go ahead and compile and build with cmake and make (instructions in SquirrelDefender).  This is the container you can use to work with the code and test it.
        8. If you just want to run the program and don't want to do any developing, then the same steps as above apply, but then you would execute
        ```
        cd ${OS_WS}/ && \
        ./OperationSquirrel/scripts/run_squirreldefender.sh
        ```
        ***Note: Understand that it's all just code.  The steps used to build on the Jetson Nano B01 can be followed almost exactly to build and compile squirreldefender on the Orin Nano.  The key differences are the object detector used on the orin (yolo), the video processing (opencv not jetson inference), gpio control (don't have yet), and we are using cuda 12.6, tensorrt 10.3, and cudnn > 8.  As long as the necessary dependencies are met it will compile.*** 

    - If you are using WSL
        1. Follow the instructions in [Building-SquirrelDefender](https://github.com/crose72/OperationSquirrel/blob/dev/SquirrelDefender/Building-SquirrelDefender.md) to compile the code in WSL.

    - If you are using Windows
        1. Install cuda 12.6, opencv 4.10.0 (https://jinscott.medium.com/build-opencv-on-windows-with-cuda-f880270eadb0), and TensorRT 10.30.0 and cuDNN 8.9.7.29 (cuDNN 9.5.1.17 had some issues that we discovered after compiling with opencv.  Runtime errors and not correctly determining the correct cuDNN version, try at your peril).
        2. Compile using CMake
        3. If you made it this far and need help, reach out on the discord.  I only have so much time to update documentation if it's not being used by anyone but me :)

        ***If you have a raspberry pi or some other linux single board computer then this code should be pretty compatible, and the process to compile it will be similar to compiling for WSL.  This is because compiling for the jetson requires some computer vision and other libraries that are jetson specific.  The code should be compatible with C++11 and C++14.  You will have to modify the CMakeLists.txt file and possibly the code to make it work.  Though a friend did get it to compile on a raspberry pi with only changing the device name for the serial port code.***

        ***If you have some other microcontroller, never fear, it's mostly just C and C++ with some standard library code (trying to limit how much).  You'll probably have to modify the CMakeLists.txt file and some of the code.***

2. Running the `squirreldefender` program with the simulation
    - Follow the instructions in [01-Setting-up-SITL](https://github.com/crose72/OperationSquirrel/blob/dev/docs/01-Setting-up-SITL.md) to setup the simulation environment.
    - Follow the instructions in [02-Connecting-WSL-and-Jetson-Nano-to-SITL](https://github.com/crose72/OperationSquirrel/blob/dev/docs/02-Connecting-WSL-and-Jetson-Nano-to-SITL.md) to connect either WSL or the Jetson to the simulation.
    - (Optional) Follow the instructions in [03-Setting-up-AirSim-with-SITL](https://github.com/crose72/OperationSquirrel/blob/dev/docs/03-Setting-up-AirSim-with-SITL.md) to setup the simulation environment.
3. Running the `squirreldefender` program with a real vehicle
    - Requires the jetson or some other single board linux computer or microcontroller capable of UART serial communication.  Configure the UART pins on your flight controller for Mavlink 2 communication.  Connect the UART pins on the jetson to the UART pins on your flight controller.  Press GO!
