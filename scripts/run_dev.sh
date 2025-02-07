export DISPLAY=:0 # or add this to your /.bashrc file and then source ~/.bashrc
xhost +

sudo docker run --runtime nvidia -it --rm --network host \
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
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume /tmp/argus_socket:/tmp/argus_socket \
  --volume /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra \
  --volume ${OS_WS}:/workspace \
  crose72/jetpack-r36.4.0:base
