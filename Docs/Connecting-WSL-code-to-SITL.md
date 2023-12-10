# Description
These instructions walk the user through how to control a simulated drone using the mavlink protocol over TCP.  It works in WSL/WSL2 and connects to the ArduPilot SITL without any additional hardware.  It will ARM the drone (if pre-arm checks pass), request some mavlink messages from the flight controller, and cause the drone to takeoff and fly in a predetermined pattern.  It is intended to simulate the code that will run on the Jetson Nano or other companion computer without needing anything but your laptop.

Some of the code is commented out intentionally, and the print functions are to help debug.

## Connect compiled executable to ArduPilot SITL on WSL/WSL2
1. Go to ArduCopter directory
	- `cd ardupilot/ArduCopter`
2. Start the SITL with desired arguments (console and map in this case)
	- `sim_vehicle.py --console --map`
3. Go to OperationSquirrel/SquirrelDefender
	- `cd OperationSquirrel/SquirrelDefender`
4. Compile the code
    - `make`
5. Execute the program on the Jetson Nano
    - `sudo ./main`
6. The drone should ARM, send back messages, and takeoff

#### This requires two instances of WSL/WSL2 to be open, one for running SITL, the other to execute the companion computer code