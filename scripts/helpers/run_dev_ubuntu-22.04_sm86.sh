#!/usr/bin/env bash
# ==============================================================================
# 🐿️ Operation Squirrel - Ubuntu 22.04 (CUDA sm86) Dev Container Launcher
# ------------------------------------------------------------------------------
# Usage:
#   ./run_dev_ubuntu22_sm86.sh              → normal interactive dev session
#   ./run_dev_ubuntu22_sm86.sh osremote     → persistent container for OSRemote
# ==============================================================================

set -e

# --------------------------------------------------------------
# Detect GitHub Actions CI
# --------------------------------------------------------------
if [ "$GITHUB_ACTIONS" = "true" ]; then
    IS_CI=true
else
    IS_CI=false
fi

MODE=${1:-plain}  # default = plain

# Workspace default
OS_WS=${OS_WS:-/home/$USER/workspaces/os-dev}

# Default display
if [ -z "$DISPLAY" ]; then
  export DISPLAY=:0
fi

xhost +local:root >/dev/null 2>&1 || true

CONTAINER_NAME="squirreldefender-dev"
IMAGE_NAME="crose72/os-dev:cuda12.6-trt10.5-cv4.10-sm86-vpi3.2-mcap-ubuntu22"
IMAGE_NAME_CICD="ghcr.io/crose72/os-dev:cuda12.6-trt10.5-cv4.10-sm86-vpi3.2-mcap-ubuntu22"

# --------------------------------------------------------------
# Device List
# --------------------------------------------------------------
DOCKER_DEVICES=(
  --device /dev/video0
  --device /dev/video1
  --device /dev/nvhost-ctrl
  --device /dev/nvhost-ctrl-gpu
  --device /dev/nvhost-prof-gpu
  --device /dev/nvmap
  --device /dev/nvhost-gpu
  --device /dev/nvhost-vic
  --device /dev/nvhost-nvdec
  --device /dev/nvhost-nvenc
  --device /dev/nvhost-msenc
  --device /dev/nvhost-isp
  --device /dev/nvhost-vi
  --device /dev/nvhost-sched
  --device /dev/nvhost-as-gpu
  --device /dev/nvhost-dbg-gpu
  --device /dev/nvhost-display
  --device /dev/nvhost-vic
  --device /dev/nvhost-nvjpg
  --device /dev/nvhost-tsec
  --device /dev/ttyUSB0
  --device /dev/ttyTHS1
)

DOCKER_VOLUMES=(
  --volume /tmp/.X11-unix:/tmp/.X11-unix
  --volume /tmp/argus_socket:/tmp/argus_socket
  --volume /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra
  --volume ${OS_WS}:/workspace
  --volume ${OS_WS}/OperationSquirrel/SquirrelDefender:/workspace/OperationSquirrel/SquirrelDefender
)

# --------------------------------------------------------------
# Pull image (if online)
# --------------------------------------------------------------
if ping -c1 -W1 8.8.8.8 &>/dev/null; then
  echo "🌐 Internet detected — pulling latest image..."
  docker pull "$IMAGE_NAME"
else
  echo "📡 Offline — skipping docker pull."
fi

# --------------------------------------------------------------
# Attach/restart existing container
# --------------------------------------------------------------
if [ "$(docker ps -aq -f name=^${CONTAINER_NAME}$)" ]; then
  if [ "$(docker ps -q -f name=^${CONTAINER_NAME}$)" ]; then
    echo "📦 Already running — attaching..."
    docker exec -it ${CONTAINER_NAME} bash -c \
      "cd /workspace/OperationSquirrel/SquirrelDefender/build && exec bash"
  else
    echo "▶️ Restarting container..."
    docker start -ai ${CONTAINER_NAME}
  fi
  exit 0
fi

# --------------------------------------------------------------
# 🚀 GitHub CI mode — non-interactive build
# --------------------------------------------------------------
if [ "$IS_CI" = true ]; then
    echo "🤖 GitHub CI detected — running non-interactive WSL build inside container..."

    docker run --rm \
        -v $GITHUB_WORKSPACE/SquirrelDefender:/workspace/OperationSquirrel/SquirrelDefender \
        $IMAGE_NAME \
        bash -c "
            cd /workspace/OperationSquirrel/SquirrelDefender &&
            mkdir -p build &&
            cd build &&
            cmake .. \
                -DBLD_WSL=ON \
                -DBLD_JETSON_B01=OFF \
                -DBLD_JETSON_ORIN_NANO=OFF \
                -DBLD_WIN=OFF \
                -DWIN_TCP=OFF \
                -DWIN_SERIAL=OFF \
                -DWIN_ENABLE_CV=OFF \
                -DBUILD_TEST_HARNESS=OFF \
                -DENABLE_GDB=OFF \
                -DENABLE_SECRET_SQUIRREL=OFF &&
            make -j\$(nproc)
        "

    exit 0
fi

# --------------------------------------------------------------
# Persistent mode (flutter OSRemote)
# --------------------------------------------------------------
if [[ "$MODE" == "osremote" ]]; then
#   echo "📱 Starting persistent OSRemote container..."
#   nohup docker run --gpus all -dit --network host \
#     --privileged --ipc=host \
#     --env DISPLAY=$DISPLAY \
#     "${DOCKER_DEVICES[@]}" \
#     "${DOCKER_VOLUMES[@]}" \
#     --name ${CONTAINER_NAME} \
#     ${IMAGE_NAME} \
#     tail -f /dev/null >/tmp/squirreldefender.log 2>&1 &

  echo "OSRemote not supported."
  exit 0
fi

# --------------------------------------------------------------
# 💻 Standard interactive dev container
# --------------------------------------------------------------
echo "💻 Launching new interactive dev container..."

docker run --gpus all -it --rm --network host \
  --add-host=host.docker.internal:host-gateway \
  --privileged --ipc=host \
  --env DISPLAY=$DISPLAY \
  "${DOCKER_DEVICES[@]}" \
  "${DOCKER_VOLUMES[@]}" \
  --name ${CONTAINER_NAME} \
  ${IMAGE_NAME} \
  bash -c "cd /workspace/OperationSquirrel/SquirrelDefender/build && exec bash"
