# operationsquirrel

### The drone
<img src="https://github.com/user-attachments/assets/e551dd46-b7c2-478f-9a46-858f54ebddc5" alt="Image 1" width="640">

### Drone stabilization test

https://github.com/user-attachments/assets/681405a0-4fe6-4277-aa67-8812df1160f3

### Autonomous drone following in simulation (hardware in loop)

https://github.com/user-attachments/assets/a6b1fb76-5357-4e33-b026-c60e2d991b9d

### Autonomous drone following in real life

My VERY FIRST test flight ever in real life.  Prior to this I had only used simulation.

https://github.com/user-attachments/assets/12bda7be-26f7-4e50-8a76-f6e40bb1b42d

Here's a better video that includes the drone's perspective too.
And I'll be uploading an even better video than this one soon!

[![Watch the video](https://img.youtube.com/vi/FgqmfTvIJS4/0.jpg)](https://youtu.be/FgqmfTvIJS4)

## Description

Operation Squirrel is an autonomous drone research project focused on visual target following, payload delivery, advanced control systems, and robust real‑world robotics engineering.

The system combines:
- CUDA accelerated YOLOv8 object detection
- Real‑time tracking + sensor fusion
- Autonomous flight behaviors
- Hardware‑in‑loop simulation
- Robust MCAP telemetry logging

The engineering work is broadly applicable to agriculture, home security, education, delivery robotics, and more.

Repository structure:
- `ardupilot` — flight controller configs, firmware, tuning
- `docker` — container definitions
- `docs` — documentation + setup guides
- `scripts` — setup + automation scripts
- `squirreldefender` — main autonomy/vision/control stack

Developer chat: <https://discord.gg/Uxg9tpVMP9>

If anything is unclear, join the Discord — we help fast :)

## Introduction (TL:DR)
 
If you want an **autonomous drone that follows a target using computer vision**, this project gives you everything.

`squirreldefender` is the Jetson‑powered companion‑computer stack that:
- Reads camera frames
- Runs high‑speed YOLOv8 detection
- Tracks a target
- Computes control commands
- Sends MAVLink to the flight controller
- Logs everything

## How you can use this project

You only need **two things**:
1. A device to run squirreldefender (Jetson, WSL2, Linux laptop)
2. A drone (real or simulated)

### Scenarios

✔ No Jetson? Use your laptop webcam with SITL
> Runs the full autonomy stack on WSL2/Linux. SITL simulates the drone.

✔ Jetson + no drone? Use SITL
> Jetson → FTDI → laptop → SITL. Camera on Jetson; flight simulated.

✔ Jetson + drone? Full real‑world autonomy
> Jetson → UART → flight controller. Camera → Jetson → squirreldefender.

✔ No Jetson + drone + FPV? Use FPV feed
> Laptop processes FPV video and sends MAVLink via SiK radio.

✔ Windows laptop + drone? Yes
> SITL + SiK radio + WSL2 + Docker.

You choose how to deploy — everything is modular.

## Getting started

The above introduction hopefully provided some context for what this project is and how you can use it.  Next I'll point you in the direction of how to get it working.

If you want to understand how the software is written and the flow, look at `docs/diagrams/software-architecture`.

- Compile & run `squirreldefender` [README](https://github.com/crose72/operationsquirrel/blob/dev/squirreldefender/README.md)
- Set up SITL Simulation [01-setting-up-sitl](https://github.com/crose72/operationsquirrel/blob/dev/docs/01-setting-up-sitl.md)
- Connecting Jetson/WSL2/Linux to SITL [02-connecting-to-sitl](https://github.com/crose72/operationsquirrel/blob/dev/docs/02-connecting-to-sitl.md)
- (Optional) AirSim photorealistic simulation [03-setting-up-airsim-with-sitl](https://github.com/crose72/operationsquirrel/blob/dev/docs/03-setting-up-airsim-with-sitl.md)
- Pre-flight checklist [Pre-Flight Checklist](https://github.com/crose72/operationsquirrel/blob/dev/docs/Pre-Flight-Checklist.md)