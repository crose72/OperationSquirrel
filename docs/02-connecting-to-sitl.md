# Part 1: Connecting WSL code to SITL

## Description

These instructions walk the user through how to control a simulated drone using the mavlink protocol over TCP.  It works in WSL/WSL2 and connects to the ArduPilot SITL without any additional hardware.  It will ARM the drone (if pre-arm checks pass), request some mavlink messages from the flight controller, and cause the drone to takeoff and fly in a predetermined pattern.  It is intended to simulate the code that will run on the Jetson Nano or other companion computer without needing anything but your laptop (or any computer running the SITL).  

***These instructions depend on [01-setting-up-sitl](https://github.com/crose72/operationsquirrel/blob/dev/docs/01-setting-up-sitl.md).  They also assume you've successfully compiled and run the squirreldefender program on WSL.***

### Cloning the operationsquirrel repo

1. Clone the repo

- `git clone --recurse-submodules https://github.com/crose72/operationsquirrel.git`

2. Update the submodules

- `git submodule init`
- `git submodule update`

### Connect compiled executable to ArduPilot SITL on WSL/WSL2

1. Go to ArduCopter directory

- `cd ardupilot/ArduCopter`

2. Start the SITL with desired arguments (console and map in this case)

- `sim_vehicle.py --console --map`

3. Go to operationsquirrel/squirreldefender

- `cd operationsquirrel/squirreldefender`

4. Compile the code
    - `mkdir build`

- `cd build`
- `cmake ..`
- `make`

5. Execute the program on the Jetson Nano
    - `sudo ./main`

- Optional: `./unit_tests` to run the unit tests

6. The drone should ARM, send back messages, and takeoff

Note: This requires two instances of WSL/WSL2 to be open, one for running SITL, the other to execute the companion computer code

# Part 2: Connecting Jetson Nano to SITL and Real drone

## Description

These instructions walk the user through how to use the Jetson Nano to control a simulated drone using the mavlink protocol over UART serial communication.  It can also be configured to work in WSL2 and connect to the ArduPilot SITL without any additional hardware (this is outlined in a "Connecting-WSL-code-to-SITL").  It will ARM the drone (if pre-arm checks pass), request some mavlink messages from the flight controller, and cause the drone to takeoff and fly in a predetermined pattern.  It should easily work for raspberry pi or other companion computers, however the UART will likely need to be configured a little differently and use the appropriate read and write functions for the desired platform.  

***These instructions depend on [01-setting-up-sitl](https://github.com/crose72/operationsquirrel/blob/dev/docs/01-setting-up-sitl.md).***

***They also assume you've successfully compiled and run the squirreldefender program on the Jetson.***

After an update to the WSL2 kernel the USBIPD feature stopped working.  Follow the instructions here if USBIPD doesn't work: <https://github.com/dorssel/usbipd-win/issues/948#issue-2290576921>.  I also recompiled the kernel to resolve the issue and my config and image files are in the `WSL` folder at the root of the repo.  Copy them to `C:\Users\<user-name>` before installing WSL2.

### Additional info

For testing purposes, run this command before executing the program on the Jetson Nano:

`sudo chmod a+rw /dev/ttyTHS1`

It temporarily enables access to the physical UART port (or whichever port you need to access) if it doesn't automatically work when executing the code.  You may be able to run the code without this command, so you may try that first.  Make sure you do this for the correct device.  Mine was ttyTHS1, yours may be different.  Additionally, it may be possible to execute the code on the Jetson Nano without doing this.  In which case, you may just execute the program (possibly needing sudo).

### Connect Jetson Nano to ArduPilot SITL on Windows WSL/WSL2

1. Connect Jetson Nano UART ports to FTDI device (3v3 logic)
2. Connect FTDI device to laptop
3. Set up USBIPD <https://learn.microsoft.com/en-us/windows/wsl/connect-usb> (first time only)
4. Open WSL2 instance
5. Check what devices are visible using windows powershell (see what bus ID the FTDI device is)

- `usbipd wsl list`


6. bind the FTDI device to WSL2 using windows powershell

- `usbipd bind --busid 2-1`

7. Attach the FTDI device to WSL2 using windows powershell

- `usbipd attach --wsl Ubuntu-22.04 --busid 2-1` (or whatever the bus id matches your device)

8. Change owner of device

- `sudo chown root:dialout /dev/ttyUSB0`

9. Check that device is added to dialout

- `ls -l /dev/ttyUSB0`

10. Give read+write access to the device

- `sudo chmod g+rw /dev/ttyUSB0`

11. Go to ArduCopter directory

- `cd ardupilot/ArduCopter`

12. Start the SITL and access the uart port that the Jetson Nano is connected to

- `sim_vehicle.py --console --map -A --serial1=uart:/dev/ttyUSB0:115200`

13. Go to operationsquirrel/squirreldefender

- `cd operationsquirrel/squirreldefender`

14. Compile the code
    - `make`
15. Execute the program on the Jetson Nano
    - `sudo ./main`
16. The drone should ARM, send back messages, and takeoff

#### If WSL2 does not recognize your device it may be due to an update in WSL2.  In which case you should follow the solution here <https://github.com/dorssel/usbipd-win/issues/948>

### Connect Jetson Nano to real flight controller

1. Connect Jetson Nano UART pins to the flight controller UART pins
2. Configure the appropriate flight controller UART port in mission planner to send and receive mavlink messages
3. Go to operationsquirrel/squirreldefender

- `cd operationsquirrel/squirreldefender`

4. Compile the code
    - `make`
5. Execute the program on the Jetson Nano
    - `sudo ./main`
6. The drone should ARM, send back messages, and takeoff
