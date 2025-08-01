# Description

Due to a update to the kernel used by WSL2 the USBIPD feature (allows you to connect usb devices to WSL2) stopped working.  In order to make it work again following the instructions here: <https://github.com/dorssel/usbipd-win/issues/948#issue-2290576921>.

I recompiled the kernel to make it work again and have included it in this folder.  Just copy the `.wslconfig` file and `bzImage-6.1` to your `C:\Users\<user-name>` path.
