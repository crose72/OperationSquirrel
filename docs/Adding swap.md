# Adding swap to your Jetson

Installing OpenCV and ORB SLAM3 require a lot of memory.  Add swap before installing them so the build doesn't fail.  Remove swap when you no longer need it.


## Installing swap
Manually install some swap (try 8GB or more to get you to 14GB)

```
sudo fallocate -l 8G /mnt/8GB.swap
sudo mkswap /mnt/8GB.swap
sudo swapon /mnt/8GB.swap
        
Then add the following line to the end of /etc/fstab to make the change persistent:
    /mnt/8GB.swap  none  swap  sw 0  0
```

(Optional if you already have enough ram) Run `sudo ./setup-swap.sh` in the `scripts` folder
- When prompted, edit the `/sbin/dphys-swapfile` so that `CONF_MAXSWAP=4096` and `CONF_SWAPFACTOR=2`
- When prompted, edit the `/etc/dphys-swapfile` so that `CONF_SWAPSIZE=4096` and `CONF_SWAPFACTOR=2`

## Removing swap

Run `sudo ./remove-swap.sh` in the `scripts` folder

Removing manually added swap
```
sudo vim /etc/fstab
    Locate the line that you added for the swap file (/mnt/4GB.swap none swap sw 0 0) and delete that line.
    Save the changes and exit the text editor.

sudo swapoff /mnt/4GB.swap
sudo rm /mnt/4GB.swap
```
