# Installing software on the Jetson-Nano

## Description

This folder contains scripts to automatically install certain dependencies needed for the project, including OpenCV, ORB SLAM3, and Jetson Inference.

## Installation instructions

1. Run ` sudo ./setup-swap.sh` to install the necessary swap needed for all builds (All of these instructions should land you about 14 GB)
    - OpenCV 4.5.0 needs about 10 GB (takes about 2 hours) and ORB SLAM3 needs about 14 GB (I went to sleep, but at least 2 hours)
      Unless you are on Jetson AGX Xavier, you should mount 4GB of swap space, as training uses up a lot of extra memory.
        - When prompted, edit the `/sbin/dphys-swapfile` so that `CONF_MAXSWAP=4096` and `CONF_SWAPFACTOR=2`
        - When prompted, edit the `/etc/dphys-swapfile` so that `CONF_SWAPSIZE=4096` and `CONF_SWAPFACTOR=2`
        ### Manually adding a litte more swap
            # Create swap file
            ***Optional*** sudo systemctl disable nvzramconfig
            sudo fallocate -l 4G /mnt/4GB.swap
            sudo mkswap /mnt/4GB.swap
            sudo swapon /mnt/4GB.swap
                
            # Then add the following line to the end of /etc/fstab to make the  change persistent:
            /mnt/4GB.swap  none  swap  sw 0  0
                
            # Unmounting swap (only do when you're done with it)
            sudo nano /etc/fstab

            # Locate the line that you added for the swap file (/mnt/4GB.swap none swap sw 0 0) and delete that line.
            # Save the changes and exit the text editor.
                
            # Deactivate the swap file you created and remove it:
            sudo swapoff /mnt/4GB.swap
            sudo rm /mnt/4GB.swap
            ***Optional*** sudo systemctl enable nvzramconfig
2. Copy `./Install-Operation-Squirrel-dependencies.sh` to your favorite code folder (if you run it in here then everything will download in here, I have a Git or GitHub folder that I use for code stuff) 
3. Reboot the Jetson Nano (included in the script)
4. Run `chmod +x ./Install-Operation-Squirrel-dependencies.sh`
5. Run `sudo ./Install-Operation-Squirrel-dependencies.sh`
6. Run `sudo ./remove-swap.sh` to stop using swap