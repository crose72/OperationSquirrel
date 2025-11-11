#!/usr/bin/env bash
# Ensure ${OS_WS} is defined
set -e

CONTAINER_NAME="squirreldefender-dev"
IMAGE_NAME="crose72/os-dev:jetpack-r36.4.0-latest"

# Pull latest image (optional but safe)
docker pull "$IMAGE_NAME"

# If container exists (either running or stopped)
if [ "$(docker ps -aq -f name=^${CONTAINER_NAME}$)" ]; then
  # If it's running, just attach
  if [ "$(docker ps -q -f name=^${CONTAINER_NAME}$)" ]; then
    echo "📦 Container '${CONTAINER_NAME}' is already running. Attaching..."
    docker exec -it ${CONTAINER_NAME} bash
  else
    echo "▶️  Restarting existing container '${CONTAINER_NAME}'..."
    docker start -ai ${CONTAINER_NAME}
  fi
  exit 0
fi

# Otherwise, run a new container
echo "🚀 Starting a new container '${CONTAINER_NAME}'..."

docker run --runtime nvidia -dit --network host \
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
  --name ${CONTAINER_NAME} \
  ${IMAGE_NAME} \
  bash -c "cd /workspace/OperationSquirrel/SquirrelDefender/build && tail -f /dev/null"
