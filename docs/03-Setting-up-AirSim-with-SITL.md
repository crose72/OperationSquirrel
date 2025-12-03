# Setting up the operationsquirrel workflow

## Description

This document will guide you through setting up AirSim to work with SITL.  The original instructions can be found here <https://ardupilot.org/dev/docs/sitl-with-airsim.html>, but I will try to simplify it after having done the process.  Note that the AirSim repository is no longer supported.  It should still work, but Codex Labs forked AirSim to a new repo called Colosseum to keep AirSim alive, and it works with Unreal Engine 5.  AirSim and Colosseum are synonymous in this document, they mean the same simulation environment, one repo is just no longer supported.  Instructions for using pre compiled binaries is also included so you can use AirSim without installing Unreal Engine or building AirSim.  

***These instructions depend on [01-Setting-up-SITL](https://github.com/crose72/operationsquirrel/blob/dev/docs/01-Setting-up-SITL.md).***

- AirSim repo: <https://github.com/microsoft/AirSim>
- Colosseum repo: <https://github.com/CodexLabsLLC/Colosseum>

## Using AirSim binaries

If you don't want to install unreal engine and build AirSim/Colosseum, then you can use precompiled binaries found here <https://github.com/microsoft/AirSim/releases>.  Just download the zip file for the environment you want to use, unzip it, and execute the application (e.g. double click `AbandonedPark.exe`)

## Setting up AirSim on Windows

1. Clone Colosseum to a location of your choosing
    - `git clone https://github.com/CodexLabsLLC/Colosseum.git`
2. Follow these instructions for installing Unreal Engine and Building Colosseum
    - <https://github.com/microsoft/AirSim/blob/main/docs/build_windows.md#build-airsim>

Note: Use the `Developer Command Prompt for VS 2022` when running `build.cmd`

Note: Replace `AirSim` with `Colosseum` when it instructs to go to the AirSim directory

3. Follow these instructions to set up the Blocks environment
    - <https://github.com/microsoft/AirSim/blob/main/docs/unreal_blocks.md>

Note: Again these steps are taking place in the `Colosseum` directory instead of AirSim because we cloned Colosseum

## WSL2: Connecting SITL to AirSim

You should already have SITL working independently before proceeding with these instructions.  AirSim is supposed to work with WSL2 according to these instructions: <https://discuss.ardupilot.org/t/gsoc-2019-airsim-simulator-support-for-ardupilot-sitl-part-ii/46395/5>, but they didn't work for me.  I did get it working by updating my `settings.json` file and using the `sim_vehicle.py` according to the post here: <https://discuss.ardupilot.org/t/gsoc-2019-airsim-simulator-support-for-ardupilot-sitl-part-ii/46395/29>

1. Set up the ArduPilot development environment according to `1.Setting-up-SITL.md`
2. Make sure you are using WSL version 2
3. Update your `settings.json` for AirSim (probably located in Documents/AirSim) to look like the following (which are found in the second link in the description above):

    ```json
    {
    "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/main/docs/settings.md",
    "SettingsVersion": 1.2,
    "LogMessagesVisible": true,
    "SimMode": "Multirotor",
    "OriginGeopoint": {
        "Latitude": -35.363261,
        "Longitude": 149.165230,
        "Altitude": 583
    },
    "Vehicles": {
        "Copter": {
            "VehicleType": "ArduCopter",
            "UseSerial": false,
            "LocalHostIp": "192.168.0.85",
            "UdpIp": "172.26.240.75",
            "UdpPort": 9003,
            "ControlPort": 9002
            }
        }
    }
    ```

    where `"LocalHostIp": "192.168.0.85",` is the IP address of your main Windows and can be found by opening a command prompt or powershell in windows and executing `ipconfig`, then look for the result under `IPv4 Address. . . . . . . . . . . : <your-windows-ip-addr>`

    and `"UdpIp": "172.26.240.75",` is the IP address of your WSL and can be found by opening an instance of WSL and executing `ip addr show eth0` and will look like this `inet <your-wsl-ip-addr>/xx`and is the number before the `/`.

4. (Instructions say to do this but I did not and it it still worked) Add the following to the end of your `~/.bashrc` file (uncomment the correct line for WSL2)

    ```
    # Export Display for XWindows
    # For WLS1
    export DISPLAY=0:0
    # For WSL2
    # export DISPLAY=$(grep -m 1 nameserver /etc/resolv.conf | awk '{print $2}'):0
    ```

5. Start AirSim by pressing the play button in VS with your Blocks or other project open (or use one of the precompiled binaries)
6. Start SITL

- `sim_vehicle.py -v ArduCopter -f airsim-copter -A --sim-address=<your-windows-ip-addr> --console --map`

7. Disable arming checks in the SITL command line (for simulation testing purposes only and only if "Main Loop" is slow)

- `param set ARMING_CHECK 0`

## WSL1: Connecting SITL to AirSim

You should already have SITL working independently before proceeding with these instructions.  AirSim is supposed to work with WSL2, these instructions are for WSL version 1, however.  These steps were done on Windows 11.  The only steps that need to be repeated should be 7 and 8 to start AirSim first and then SITL afterwards.  I have noticed that when I restart WSL I need to do step 5 again to apply the changes to the bash configuration, and VcXsrv does need to be opened again after restarting your computer

1. Set up the ArduPilot development environment according to `1.Setting-up-SITL.md`
2. Make sure you are using WSL version 1
3. Update your `settings.json` for AirSim (probably located in Documents/AirSim) to look like the following:

```json
{
"SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/main/docs/settings.md",
"SettingsVersion": 1.2,
"LogMessagesVisible": true,
"SimMode": "Multirotor",
"OriginGeopoint": {
    "Latitude": -35.363261,
    "Longitude": 149.165230,
    "Altitude": 583
},
"Vehicles": {
    "Copter": {
        "VehicleType": "ArduCopter",
        "UseSerial": false,
        "LocalHostIp": "127.0.0.1",
        "UdpIp": "127.0.0.1",
        "UdpPort": 9003,
        "ControlPort": 9002
        }
    }
}
```

4. Add the following to the end of your `~/.bashrc` file

```
# Export Display for XWindows
# For WLS1
export DISPLAY=0:0
# For WSL2
# export DISPLAY=$(grep -m 1 nameserver /etc/resolv.conf | awk '{print $2}'):0
```

5. Apply the changes to the bash configuration
    - Use `source ./.bashrc` OR `source ~/.bashrc` OR `. ~/.bashrc`

###### Might have to do this each time you close and reopen WSL

6. Open `VcXsrv` with the following options selected
    - Select display settings: “Multiple Windows” but it is your preference
    - Select how to start clients: Must select “Start no Client”
    - Extra settings: Must check “Disable access control”

Even if you're on Windows 11 - which natively supports x server displays in WSL2 - because we are in WSL version 1 we need this 3rd party x server client

7. Start AirSim by pressing the play button in VS with your Blocks or other project open
8. Start SITL
    - `sim_vehicle.py -v ArduCopter -f airsim-copter -A --sim-address=$WSL_HOST_IP --console --map`
9. Disable arming checks in the SITL command line (for simulation testing purposes only and only if "Main Loop" is slow)
    - `param set ARMING_CHECK 0`

## Additional notes

Somebody else had to use a slightly modified version of the command in step 8 when starting SITL
    - `sim_vehicle.py -v ArduCopter -f airsim-copter -A --sim-address=169.254.255.255 --console --map`
    - Where `169.254.255.255` sim address was found using the command `ip addr show eth0`.  Below is an example of what that output might look like and the value to choose is the one with enclosed with **.  (it may change sometimes, may need to check it occasionally)

```
12: eth0: <> mtu 1500 group default qlen 1
    link/ether 9c:2d:cd:3f:8e:98
    inet 169.254.217.14/16 brd *169.254.255.255* scope global dynamic
       valid_lft forever preferred_lft forever
    inet6 fe80::c97a:e882:8725:e603/64 scope link dynamic
       valid_lft forever preferred_lft forever
```
