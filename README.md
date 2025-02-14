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

- `ardupilot` - flight controller parameters, firmware, and other ardupilot related things
- `docker` - dockerfiles for building containers
- `docs` - instructions on how to set up the simulation and code documentation
- `scratch` - sandbox for experimental code worth saving
- `scripts` - scripts to automatically install dependencies or do other tasks
- `SquirrelDefender` - autonomous drone code for companion computer or ground station
- `tools` - tools for processing data or performing other useful tasks
- `wsl` - WSL2 kernel that supports USBIPD (connecting usb devices to wsl)

Developer chat: <https://discord.gg/Uxg9tpVMP9>

If you need help or the instructions are unclear, please reach out in the discord  :)

## Introduction (TL:DR)
 
Some things you might be wondering about this project are "How do I use this?", "What does the code do?"  If you want to make an autonomous drone that is able to identify and follow a target based an object detector that is performing inference on a live camera feed, then the code in the SquirrelDefender folder is what you want.  

You need two things for this project to work for you.  First you need a device on which to compile the code (Jetson Nano, Jetson Orin Nano, Windows laptop, microcontroller, etc).  Second you need a drone (real or simulated).

If you don't have a Jetson and never will, not to fear, you can still compile the `squirreldefender` code on your laptop, run the simulation, and it will control the simulated drone based on your webcam feed.

If you have a Jetson or other SBC but you don't have a drone, don't worry, you can compile the `squirreldefender` code on the jetson, connect the jetson to your laptop via FTDI to USB device, run the simulation on the laptop, and then run the program on the jetson and it will control the drone based on the camera feed from your jetson.

If you have a Jetson and a drone, hooray!  Same thing as above but now you connect the UART pins on your Jetson to the UART on your drone, which should be configured for Mavlink 2.

What if you don't have a Jetson but you have a drone, hooray!  If you have an fpv camera on your drone and are able to stream that back to your laptop (or technically the Jetson if you didn't want to mount the Jetson onto the drone), then you can follow a target using the fpv camera feed and as long as you have a SiK radio or something similar to send out the mavlink commands to the drone, this will work.

What if you don't have a Jetson but you have a drone and a windows laptop, hooray!  Well, sort of.  If you have a SiK radio connected to one of the USB ports on your laptop you can actually send commands to the drone with our code that way too (though mission planner and q grount control can do that too).

Okay now that I've told you many ways in which you can use this project I'll leave it up to you to decide how to use it for your needs.

## Getting started

The above introduction hopefully provided some context for what this project is and how you can use it.  Next I'll point you in the direction of how to get it working.

If you want to understand how the software is written and the flow, look at `docs/diagrams/software-architecture`.

- Follow the instructions in [README](https://github.com/crose72/OperationSquirrel/blob/dev/SquirrelDefender/README.md) to compile and run the program
- Follow the instructions in [01-Setting-up-SITL](https://github.com/crose72/OperationSquirrel/blob/dev/docs/01-Setting-up-SITL.md) to setup the simulation environment.
- Follow the instructions in [02-Connecting-to-SITL](https://github.com/crose72/OperationSquirrel/blob/dev/docs/02-Connecting-to-SITL.md) to control the simulated drone with your Jetson
- (Optional) Follow the instructions in [03-Setting-up-AirSim-with-SITL](https://github.com/crose72/OperationSquirrel/blob/dev/docs/03-Setting-up-AirSim-with-SITL.md) to setup a photo-realistic simulation
- If using real drone then configure the UART pins on your flight controller for Mavlink 2 communication and connect the UART pins on the Jetson to the UART pins on your flight controller.