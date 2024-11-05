# OperationSquirrel

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

# Getting started

1. Compiling the `squirreldefender` program
    - If you DO have the Jetson Nano or Jetson Orin Nano
        1. Follow the instructions in [Install-dependencies](https://github.com/crose72/OperationSquirrel/blob/dev/scripts/Install-dependencies.md) to setup swap and install the dependencies.
        2. Folow the instructions in [Building-SquirrelDefender](https://github.com/crose72/OperationSquirrel/blob/dev/SquirrelDefender/Building-SquirrelDefender.md) to compile the code on the Jetson.
    - If you DO NOT have a Jetson and are using WSL
        1. Follow the instructions in [Building-SquirrelDefender](https://github.com/crose72/OperationSquirrel/blob/dev/SquirrelDefender/Building-SquirrelDefender.md) to compile the code in WSL.

        ***If you have a raspberry pi or some other linux single board computer then this code should be pretty compatible, and the process to compile it will be similar to compiling for WSL.  This is because compiling for the jetson requires some computer vision and other libraries that are jetson specific.  The code should be compatible with C++11 and C++14.  You will have to modify the CMakeLists.txt file and possibly the code to make it work.  Though a friend did get it to compile on a raspberry pi with only changing the device name for the serial port code.***

        ***If you have some other microcontroller, never fear, it's mostly just C and C++ with some standard library code (trying to limit how much).  You'll probably have to modify the CMakeLists.txt file and some of the code.***

2. Running the `squirreldefender` program with the simulation
    - Follow the instructions in [01-Setting-up-SITL](https://github.com/crose72/OperationSquirrel/blob/dev/docs/01-Setting-up-SITL.md) to setup the simulation environment.
    - Follow the instructions in [02-Connecting-WSL-and-Jetson-Nano-to-SITL](https://github.com/crose72/OperationSquirrel/blob/dev/docs/02-Connecting-WSL-and-Jetson-Nano-to-SITL.md) to connect either WSL or the Jetson to the simulation.
    - (Optional) Follow the instructions in [03-Setting-up-AirSim-with-SITL](https://github.com/crose72/OperationSquirrel/blob/dev/docs/03-Setting-up-AirSim-with-SITL.md) to setup the simulation environment.
3. Running the `squirreldefender` program with a real vehicle
    - Requires the jetson or some other single board linux computer or microcontroller capable of UART serial communication.  Configure the UART pins on your flight controller for Mavlink 2 communication.  Connect the UART pins on the jetson to the UART pins on your flight controller.  Press GO!
