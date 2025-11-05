#!/bin/bash

# Define log directory on Jetson
LOG_DIR="/home/crose72/logs/"

# Ensure the log directory exists
mkdir -p "$LOG_DIR"

export DISPLAY=:0
export XDG_RUNTIME_DIR=/run/user/1000

CONTAINER_NAME="squirreldefender"

if sudo docker ps --filter "name=$CONTAINER_NAME" --format '{{.Names}}' | grep -q "^$CONTAINER_NAME$"; then
    echo "Container '$CONTAINER_NAME' is already running."
    exit 0
fi

# If an old stopped container exists, remove it
if sudo docker ps -a --filter "name=$CONTAINER_NAME" --format '{{.Names}}' | grep -q "^$CONTAINER_NAME$"; then
    echo "Removing old container '$CONTAINER_NAME'"
    sudo docker rm $CONTAINER_NAME
fi

# Start a new container
echo "Starting container '$CONTAINER_NAME'"

sudo docker run --rm --runtime nvidia --network host \
  --privileged --ipc=host \
  --env DISPLAY=$DISPLAY \
  --env XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
  --env NVIDIA_DRIVER_CAPABILITIES=all \
  --env NVIDIA_VISIBLE_DEVICES=all \
  --device /dev/video0 \
  --device /dev/video1 \
  --device /dev/nvhost-ctrl \
  --device /dev/nvhost-ctrl-gpu \
  --device /dev/nvhost-prof-gpu \
  --device /dev/nvmap \
  --device /dev/nvhost-gpu \
  --device /dev/nvhost-vic \
  --device /dev/nvhost-nvdec \
  --device /dev/nvhost-nvenc \
  --device /dev/nvhost-msenc \
  --device /dev/nvhost-isp \
  --device /dev/nvhost-vi \
  --device /dev/nvhost-sched \
  --device /dev/nvhost-as-gpu \
  --device /dev/nvhost-dbg-gpu \
  --device /dev/nvhost-display \
  --device /dev/nvhost-vic \
  --device /dev/nvhost-nvjpg \
  --device /dev/nvhost-tsec \
  --device /dev/nvhost-isp \
  --device /dev/ttyUSB0 \
  --device /dev/ttyTHS1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume /tmp/argus_socket:/tmp/argus_socket \
  --volume /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra \
  --volume /dev/dri:/dev/dri \
  --volume /run:/run \
  -v "$LOG_DIR":/workspace/OperationSquirrel/SquirrelDefender/data/ \
  --name $CONTAINER_NAME \
  crose72/jetson-inference-r32.7.1:squirreldefender
