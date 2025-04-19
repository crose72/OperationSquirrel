# Ensure ${OS_WS} is defined
DOCKER_IMG="crose72/jetpack-r36.4.0:base"
DOCKER_WS="/workspace/OperationSquirrel"
docker pull ${DOCKER_IMG}
sudo docker run --runtime nvidia --rm -it --network host \
  --privileged --ipc=host \
  --env DISPLAY=$DISPLAY \
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
  --volume ${OS_WS}/OperationSquirrel/SquirrelDefender:/workspace/OperationSquirrel/SquirrelDefender \
  --volume ${OS_WS}/YOLOv8-TensorRT-CPP:/workspace/YOLOv8-TensorRT-CPP \
  --volume ${OS_WS}/OperationSquirrel/scripts:/workspace/OperationSquirrel/scripts \
  --name squirreldefender-dev \
  ${DOCKER_IMG} \
  bash -c "cd '${DOCKER_WS}'/SquirrelDefender/build \
    && export DOCKER_IMG='${DOCKER_IMG}' \
    && export OS_WS='${DOCKER_WS}' \
    && export PATH='${DOCKER_WS}'/scripts:\$PATH \
    && echo hello! \
    && exec bash" \

# Show running containers after starting the container
#echo "Running Docker containers:"
#docker ps