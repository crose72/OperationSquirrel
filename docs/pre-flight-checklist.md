# Pre-Flight Checklist

## Description

This document will outline high level steps that should be followed to prepare for a test flight with a real drone.  It is assumed that you have successfully completed the instructions in [README](https://github.com/crose72/operationsquirrel/blob/dev/squirreldefender/README.md) to have a compiled squirreldefender program, and have configured your Jetson to run the squirreldefender program as soon as you power it on.  

When you go to do autonomous flight you're probably going to go to a field with the drone and set it on the ground, and then plug in the battery to power the drone.  Then, with Jetson mounted onto your drone somehow, you are going to power the Jetson (possibly with a barrel jack connected to the drone's LiPo battery) and will probably want to have the drone takeoff and start following you without needing your laptop or a computer monitor in order to start the `squirreldefender` program.  Thus, you need to make sure that the program start as soon as you turn on the Jetson, and the instructions for that are in [README](https://github.com/crose72/operationsquirrel/blob/dev/squirreldefender/README.md).

These instructions are written with ArduPilot in mind, but any drone that uses mavlink should work with this code (such as iNav or PX4).

## Preparing for flight

### squirreldefender Setup

1. squirreldefender program compiled [README](https://github.com/crose72/operationsquirrel/blob/dev/squirreldefender/README.md)
2. squirreldefender program runs when Jetson is powered on [README](https://github.com/crose72/operationsquirrel/blob/dev/squirreldefender/README.md)
3. Set desired takeoff height in `vehicle_controller.cpp` (default is ~4-7m)
4. Set desired follow distance in `follow_target.cpp` (if you have a reason to change it)
5. Set PID gains in `follow_target.cpp` (if you have a reason to change them)
6. Verify the squirreldefender program in SITL
    - It's helpful to make sure that at least the mavlink communication is working and when you start the program you see the drone start moving left/right/forward based on when you are in from of the camera
    - Any sanity check you need to be confident it will work on the real drone

### Drone Setup

1. Configure a UART on the flight controller for MAVLINK 2.0 from a companion computer at the same BAUD as the Jetson (115200 is what the Jetson is currently set at)
2. Verify Stabilize mode works on the drone
    - This ensures that motor directions are all correct and the drone is generally working
2. Verify Loiter mode works on the drone
    - This ensures that the drone can successfully hold it's position and that the GPS fix is good and healthy (our drone uses GUIDED mode which depends on GPS, if Loiter doesn't work then GUIDED won't work)

## Flying

1. Make sure you are the only person the drone identifies
    - Nobody else should be visible by the drone's camera
    - Currently if multiple targets are seen it will try to follow the last one it sees, which can be problematic if multiple people are visible to the camera
2. Place your drone on the ground away from any obstacles (trees, buildings, etc)
3. Connect UART pins on the drone to the UART pins on the Jetson
4. Power on the drone
5. Verify that the drone will arm in Loiter mode (remember we need a good and healthy GPS fix)
6. Power on the Jetson and walk away
    - You or someone else should have a transmitter and be ready to take control of the drone at anytime should something go wrong
7. After the drone has achieved the desired takeoff height you can walk into frame (in view of the camera)
    - The drone will now follow you, enjoy :)
