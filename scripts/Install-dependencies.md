# Installing SquirrelDefender depenencies

## Description

This folder contains scripts to automatically install certain dependencies needed for the project, including OpenCV, ORB SLAM3, Jetson Inference, and more on the Jetson Nano and the Jetson Orin Nano.  It should also work for other jetson platforms but has not been tested on any other ones.

## Installation instructions for Jetson Nano B01, A01, A02

1. First manually install some swap (try 8GB or more to get you to 14GB)

    ```
    sudo fallocate -l 4G /mnt/4GB.swap
    sudo mkswap /mnt/4GB.swap
    sudo swapon /mnt/4GB.swap
            
    Then add the following line to the end of /etc/fstab to make the change persistent:
        /mnt/4GB.swap  none  swap  sw 0  0
    ```

2. (Optional if you already have enough ram) Run `sudo ./setup-swap.sh`
    - OpenCV 4.5.0 needs about 10 GB (takes about 2 hours) and ORB SLAM3 needs about 14 GB
      Unless you are on Jetson AGX Xavier, you should mount 4GB of swap space, as training uses up a lot of extra memory.
        - When prompted, edit the `/sbin/dphys-swapfile` so that `CONF_MAXSWAP=4096` and `CONF_SWAPFACTOR=2`
        - When prompted, edit the `/etc/dphys-swapfile` so that `CONF_SWAPSIZE=4096` and `CONF_SWAPFACTOR=2`
3. Copy `./Jetson-Nano-Install.sh` to your favorite code folder (if you run it in here then everything will download in here, I have a Git or GitHub folder that I use for code stuff)
4. Reboot the Jetson Nano (included in the script)
5. Uninstall default OpenCV version `sudo apt-get remove --purge *libopencv*`
6. Run `chmod +x ./Jetson-Nano-Install.sh`
7. Run `sudo ./Jetson-Nano-Install.sh`
8. Unzip and copy the contents of `jetson/usr-b01-nano` from this repository to 'the respective folders in `usr/local`
    ```
    sudo cp -r ./usr-b01-nano/local/include/* /usr/local/include/
    sudo cp ./usr-b01-nano/local/lib/* /usr/local/lib/
    ```
    - What this does is move the library and header files to a location where they can be accessed by the whole computer.  You are only copying the files for Pangolin and ORB_SLAM3 because they do not have a make install recipe which automatically installs them to these locations.  I also had to manually move the header files for SLAM to make sure that code compiled when written outside of the ORB_SLAM3 folder.
9. Run `sudo ldconfig`

## Installation instructions for Jetson Orin Nano 8GB with Jetpack 5 and Ubuntu-20.04

1. First manually install some swap

    ```
    sudo fallocate -l 8G /mnt/8GB.swap
    sudo mkswap /mnt/8GB.swap
    sudo swapon /mnt/8GB.swap
            
    Then add the following line to the end of /etc/fstab to make the change persistent:
        /mnt/8GB.swap  none  swap  sw 0  0
    ```

2. (Optional if you already have enough ram) Run `sudo ./setup-swap.sh`
    - OpenCV 4.6.0 needs about 10 GB (takes about 2 hours) and ORB SLAM3 needs about 14 GB
      Unless you are on Jetson AGX Xavier, you should mount 4GB of swap space, as training uses up a lot of extra memory.
        - When prompted, edit the `/sbin/dphys-swapfile` so that `CONF_MAXSWAP=4096` and `CONF_SWAPFACTOR=2`
        - When prompted, edit the `/etc/dphys-swapfile` so that `CONF_SWAPSIZE=4096` and `CONF_SWAPFACTOR=2`
3. Copy `./Jetson-Orin-Install.sh` to your favorite code folder (if you run it in here then everything will download in here, I have a Git or GitHub folder that I use for code stuff)
4. Reboot the Jetson Nano (included in the script)
5. Uninstall default OpenCV version `sudo apt-get remove --purge *libopencv*`
6. Run `chmod +x ./Jetson-Orin-Install.sh`
7. Run `sudo ./Jetson-Orin-Install.sh`
8. Unzip and copy the contents of `jetson/usr-orin-nano` from this repository to the respective folders in `usr/local`
    ```
    sudo cp -r ./usr-orin-nano/local/include/* /usr/local/include/
    sudo cp ./usr-orin-nano/local/lib/* /usr/local/lib/
    ```
    - What this does is move the library and header files to a location where they can be accessed by the whole computer.  You are only copying the files for Pangolin and ORB_SLAM3 because they do not have a make install recipe which automatically installs them to these locations.  I also had to manually move the header files for SLAM to make sure that code compiled when written outside of the ORB_SLAM3 folder.
9. Run `sudo ldconfig`

***If you use a different version of OpenCV then you will need to manually copy your ORB_SLAM3 headers to the `usr/local/lib` path from where they were created instead of the `jetson` folder in this repository.  This is because they will be linked to different versions of OpenCV

## Remove swap

1. Run `sudo ./remove-swap.sh`
2. Removing manually added swap

    ```
    sudo vim /etc/fstab
        Locate the line that you added for the swap file (/mnt/4GB.swap none swap sw 0 0) and delete that line.
        Save the changes and exit the text editor.
    
    sudo swapoff /mnt/4GB.swap
    sudo rm /mnt/4GB.swap
    ```
