# Setting up the Operation Squirrel workflow

## Summary

This document will guide you through setting up the entire workflow needed for development, including setting up the ArduPilot SITL, using the SITL, and connecting to SITL with the companion computer code.  The complete instructions for setting up SITL are here https://ardupilot.org/dev/docs/building-setup-windows10.html#building-setup-windows10 but it can be a little confusing because it takes you to some other pages so then you’ll have to follow those steps and come back to that link.  Hopefully this will make things less confusing.  But if this guide fails you can always refer to the original documentation.  This page only shows how to set up the environment, not how to run the SITL.  But I will include a link to basic SITL usage.  These instructions are stand alone.

## Setting up the ArduPilot build environment

1. Install WSL2 (Ubuntu-20.04 is what we’re using) 
    - https://learn.microsoft.com/en-us/windows/wsl/install
    - Be sure you’re not root when doing everything.  You should have to use the “sudo” command for some things.        If your WSL2 terminal is a little weird (maybe a little gray) or you never need to “sudo” anything (get permission) then you may be root and need to remove WSL2 and reinstall it.
        - You should fine if you set up a username and password.
2. Set up the ArduPilot build environment
    - https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux
    - You don’t need the git gui and WSL2 should come with git.  You should only need to follow the instructions        to clone the repo and install the required packages with the shell script.
3. Confirm that your build environment is set up properly
    - https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md
    - In WSL2 just change directories to the ardupilot/ArduCopter directory (wherever you cloned it)
        - `cd ardupilot/ArduCopter`
    - Configure with waf
        - `./waf configure`
    - Build with waf
        - `./waf copter`
    - It might take a few minutes but shouldn’t be too long.  You’ll see the files that are compiled.
4. Xserver setup
    - https://ardupilot.org/dev/docs/building-setup-windows10.html#building-setup-windows10
        - Look at the **Using SITL with WSL and XWindows (Windows 10 only)** heading and **Using VcXsrv to Create           XWindows Displays** sections towards the bottom. 
    - This page may be helpful in setting up the linux display windows
        - https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps
    - This page helped me update the linux kernel, which was the fix I needed in order to have my “out of the box”       xserver windows work in windows 11.  You would think that WSL2 would automatically update the latest kernel         but mine didn’t.  You may not need to do this, but it is there as a solution to an issue I had.
        - https://learn.microsoft.com/en-us/windows/wsl/install-manual#step-4---download-the-linux-kernel-update-package

## Start the simulation

1. First time setup of SITL
    - https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html#setting-up-sitl-on-linux
        - Look under **Start SITL simulator**.  It will have you execute the simulation twice, first with the `-w` flag to clear the virtual EEPROM.  You only need to do this on the first set up and if you want to wipe the parameters.
2. SITL tutorial
    - https://ardupilot.org/dev/docs/copter-sitl-mavproxy-tutorial.html
        - This will help you just get the thing up in the air so you can see it move.  Very basic but it's the "hello world" of ArduPilot SITL.

You have now successfully setup SITL
        
## How to use our code to communicate with SITL

Read [02-Connecting-WSL-code-to-SITL](https://github.com/crose72/OperationSquirrel/blob/master/Docs/02-Connecting-WSL-code-to-SITL.md) and [03-Connecting-Jetson-Nano-to-SITL-and-real-drone](https://github.com/crose72/OperationSquirrel/blob/master/Docs/03-Connecting-Jetson-Nano-to-SITL-and-real-drone.md) to learn how to use control a simulated or real drone with the code.
