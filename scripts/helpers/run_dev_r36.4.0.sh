#!/usr/bin/env bash
# ==============================================================
# 🐿️ Operation Squirrel - Jetson Container Launcher
# --------------------------------------------------------------
# Usage:
#   ./run_dev.sh              → normal dev session
#   ./run_dev.sh osremote     → persistent container for Flutter remote
# ==============================================================

set -e

# Detect GitHub Actions CI
if [ "$GITHUB_ACTIONS" = "true" ]; then
    IS_CI=true
else
    IS_CI=false
fi

MODE=${1:-plain}  # default = plain
OS_WS=${OS_WS:-/home/$USER/workspaces/os-dev}

# Default display if not set
if [ -z "$DISPLAY" ]; then
  export DISPLAY=:0
fi

xhost +local:root >/dev/null 2>&1 || true

CONTAINER_NAME="squirreldefender-dev"
IMAGE_NAME="crose72/os-dev:jetpack-r36.4.0-latest"

# --------------------------------------------------------------
# Common Docker configuration
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
  --volume ${OS_WS}/OperationSquirrel/SquirrelDefender:/workspace/OperationSquirrel/SquirrelDefender
)

# --------------------------------------------------------------
# Optional image pull (only if online)
# --------------------------------------------------------------
if ping -c1 -W1 8.8.8.8 &>/dev/null; then
  echo "🌐 Internet detected — pulling latest image..."
  docker pull "$IMAGE_NAME"
else
  echo "📡 Offline mode — skipping docker pull."
fi

# --------------------------------------------------------------
# 🔁 Universal attach/restart logic
# --------------------------------------------------------------
if [ "$(docker ps -aq -f name=^${CONTAINER_NAME}$)" ]; then
  if [ "$(docker ps -q -f name=^${CONTAINER_NAME}$)" ]; then
    echo "📦 Container '${CONTAINER_NAME}' is already running — attaching..."
    docker exec -it ${CONTAINER_NAME} bash -c "cd /workspace/OperationSquirrel/SquirrelDefender/build && exec bash"
  else
    echo "▶️  Restarting stopped container '${CONTAINER_NAME}'..."
    docker start -ai ${CONTAINER_NAME}
  fi
  exit 0
fi

# --------------------------------------------------------------
# 🚀 Launch a new container
# --------------------------------------------------------------
if [ "$IS_CI" = true ]; then
    echo "🤖 GitHub CI detected — running non-interactive container build..."

    docker run --rm \
        -v ${OS_WS}/OperationSquirrel/SquirrelDefender:/workspace/OperationSquirrel/SquirrelDefender \
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
elif [[ "$MODE" == "osremote" ]]; then
  echo "📱 Launching persistent container for OSRemote (Flutter app)..."
  nohup docker run --runtime nvidia -dit --network host \
    --privileged --ipc=host \
    --env DISPLAY=$DISPLAY \
    "${DOCKER_DEVICES[@]}" \
    "${DOCKER_VOLUMES[@]}" \
    --name ${CONTAINER_NAME} \
    ${IMAGE_NAME} \
    tail -f /dev/null > /tmp/squirreldefender.log 2>&1 &
  echo "✅ Persistent OSRemote container started in background."
else
  echo "💻 Launching new interactive dev container..."
  docker run --runtime nvidia -it --rm --network host \
    --privileged --ipc=host \
    --env DISPLAY=$DISPLAY \
    "${DOCKER_DEVICES[@]}" \
    "${DOCKER_VOLUMES[@]}" \
    --name ${CONTAINER_NAME} \
    ${IMAGE_NAME} \
    bash -c "cd /workspace/OperationSquirrel/SquirrelDefender/build && exec bash"
fi