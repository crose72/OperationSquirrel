# SIL notes

### Starting the SIL with a real serial device (e.g. connecting jetson) and custom parameters

`sim_vehicle.py --console --map -A --serial1=uart:/dev/ttyUSB0:115200 --add-param-file=/home/crose72/Documents/GitHub/OperationSquirrel/ArduPilot/SITL_params/copter-OF-RF.parm`

- Use .parm files for ardupilot and .params for qgroundcontrol

