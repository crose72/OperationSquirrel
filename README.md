# OperationSquirrel

This project is aimed at creating an autonomous drone to defend homes from squirrels.  Advanced control methods, path planning algorithms, localization, and AI/machine learning techniques will be deployed to achieve this goal.  The scope of this project is narrow, but the applications vary widely, as all of the engineering feats that need to be accomplished along the way are stepping stones for other exciting problems.  The work we do for this project can be expanded to defend crops from birds, home security, deliveries, or any other creative use you can come up with.

- `ArduPilot` - parameters, firmware and any other ardupilot related things
- `Docs` - instructions on how to set up the workflow and various other things
- `Hardware` - documentation for hardware
- `Jetson` - specific libraries, headers, and other files already organized for you :)
- `scratch` - miscellaneous experimental code, a sandbox to try out code and save it to the repo
- `scripts` - scripts to automatically install dependencies or do other tasks
- `SquirrelDefender` - the main application code
- `ThirdParty` - third party software included as submodules
- `tools` - tools specific to operation squirrel for processing data or performing other useful tasks

Developer chat: <https://discord.gg/Uxg9tpVMP9>

# Getting started

1. Compiling the code
    - If you DO have the Jetson Nano
        1. Follow the README.md in `scripts` to setup swap and install the dependencies.
        2. Folow the README.md in `SquirrelDefender` to compile the code for the Jetson.
    - If you DO NOT have a Jetson and are using WSL
        1. Follow the instructions in `Docs` to setup SITL.
            - This will give you WSL and ArduPilot SITL
        2. Follow the instructions in `SquirrelDefender` to compile the code for WSL.
2. Running the program with the simulation
    - If you DO have the Jetson Nano
        1. Follow the instructions in `Docs` to connect the Jetson to WSL2.
    - If you DO NOT have a Jetson and are using WSL
        1. Follow the instructions in `Docs` to connect the program in WSL to SITL
3. Running the program with a real vehicle
    - Requires the Jetson nano.  Connect UART pins on the Jetson to the configured UART pins on your flight controller.
