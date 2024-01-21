# Summary

These instructions walk the user through how to use the Jetson Nano to control a simulated drone using the mavlink protocol over UART serial communication.  It can also be configured to work in WSL2 and connect to the ArduPilot SITL without any additional hardware (this is outlined in a "Connecting-WSL-code-to-SITL").  It will ARM the drone (if pre-arm checks pass), request some mavlink messages from the flight controller, and cause the drone to takeoff and fly in a predetermined pattern.  It should easily work for raspberry pi or other companion computers, however the UART will likely need to be configured a little differently and use the appropriate read and write functions for the desired platform.  These instructions depend on [01-Setting-up-the-workflow](https://github.com/crose72/OperationSquirrel/blob/master/Docs/01-Setting-up-the-workflow.md).

## Additional info

For testing purposes, run this command before executing the program on the Jetson Nano:

`sudo chmod a+rw /dev/ttyTHS1`

It temporarily enables access to the physical UART port (or whichever port you need to access) if it doesn't automatically work when executing the code.  You may be able to run the code without this command, so you may try that first.  Make sure you do this for the correct device.  Mine was ttyTHS1, yours may be different.  Additionally, it may be possible to execute the code on the Jetson Nano without doing this.  In which case, you may just execute the program (possibly needing sudo).


## Connect Jetson Nano to ArduPilot SITL on Windows WSL/WSL2

1. Connect Jetson Nano UART ports to FTDI device (3v3 logic)
2. Connect FTDI device to laptop
3. Set up USBIPD https://learn.microsoft.com/en-us/windows/wsl/connect-usb (first time only)
4. Check what devices are visible using windows powershell (see what bus ID the FTDI device is)
	- `usbipd wsl list`
5. Open WSL2 instance
6. Attach the FTDI device to WSL2 using windows powershell
	- `usbipd wsl attach --busid 2-1` (or whatever the bus id matches your device)
7. Change owner of device
	- `sudo chown root:dialout /dev/ttyUSB0`
8. Check that device is added to dialout
	- `ls -l /dev/ttyUSB0`
9. Give read+write access to the device
	- `sudo chmod g+rw /dev/ttyUSB0`
10. Go to ArduCopter directory
	- `cd ardupilot/ArduCopter`
11. Start the SITL and access the uart port that the Jetson Nano is connected to
	- `sim_vehicle.py --console --map -A --serial2=uart:/dev/ttyUSB0:115200`
12. Go to OperationSquirrel/SquirrelDefender
	- `cd OperationSquirrel/SquirrelDefender`
13. Compile the code
    - `make`
14. Execute the program on the Jetson Nano
    - `sudo ./main`
15. The drone should ARM, send back messages, and takeoff

## Connect Jetson Nano to real flight controller

1. Connect Jetson Nano UART pins to the flight controller UART pins
2. Configure the appropriate flight controller UART port in mission planner to send and receive mavlink messages
3. Go to OperationSquirrel/SquirrelDefender
	- `cd OperationSquirrel/SquirrelDefender`
4. Compile the code
    - `make`
5. Execute the program on the Jetson Nano
    - `sudo ./main`
6. The drone should ARM, send back messages, and takeoff