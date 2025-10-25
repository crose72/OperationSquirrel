# Ensure ${OS_WS} is defined
docker pull crose72/os-dev:cuda11.8-cudnn89-trt853-mcap-ubuntu22
sudo docker run --gpus all -it --rm --network host \
  --add-host=host.docker.internal:host-gateway \
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
  --volume ${OS_WS}:/workspace \
  --volume ${OS_WS}/OperationSquirrel/SquirrelDefender:/workspace/OperationSquirrel/SquirrelDefender \
  --name squirreldefender-dev \
  crose72/os-dev:cuda11.8-cudnn89-trt853-mcap-ubuntu22 \
  bash -c "cd /workspace/OperationSquirrel/SquirrelDefender/build \
    && exec bash"
