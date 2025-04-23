# Scripts

## Description

This folder contains scripts that perform useful tasks, such as installing a particular version a CMake, installing the dependencies needed 
to compile the program on your jetson, and building and running the docker containers which can be used for developing and running the program.  Descriptions of only a few of the scripts will be included below.  

Please note that if you are building the containers using these scripts you will not be able to push to my repo, so if you want to create your own container then you need to changes the username of the docker hub repo you're pushing to.

The `Install-Jetson-<rest of name here>` scripts Are used for installing SLAM, OpenCV, Jetson-Inference, and a few other things that were needed for the program.  It was used when developing on the Jetson itself, but docker containers have simplified that workflow so you don't have to install those things yourself, you just run the container :) They are a little dated, but they should work if you need them.

The dev container builds opencv from a script, and that script needs to know which Jetson is being used so it can compile opencv for the correct GPU generation/architecture.  So before building dev for the first time you need to copy the device tree model to the `docker/fake-proc/device-tree/` folder before building the container, like so.

```
sudo cp /proc/device-tree/model <path-to-OperationSquirrel-repo>/docker/fake-proc/device-tree/model
```

## The Scripts

- `build_dev_r36.4.0.sh` - builds Jetpack r36.4.0 based container with all dependencies needed to compile and run the program
- `build_squirreldefender_r36.4.0.sh` - builds Jetpack r36.4.0 based container to run the pre-compiled squirreldefender program
- `build_field_r36.4.0.sh` - builds Jetpack r36.4.0 based container with code built into the container
- `run_dev_orin.sh` - runs Jetpack r36.4.0 based container with mounted code to develop, compile, and run on the Jetson Orin Nano
- `run_field_orin.sh` - runs Jetpack r36.4.0 based container with code built in to develop, compile, and run on the Jetson Orin Nano
- `run_squirreldefender_orin.sh` - runs Jetpack r36.4.0 based container which immediately runs the squirreldefender program