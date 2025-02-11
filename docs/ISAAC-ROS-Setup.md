# Instructions for setting up ISAAC_ROS on Jetson Orin Nano

## Add environment setup commands here
echo "Setting up environment..."

cat /etc/nv_tegra_release

## Install Jetpack components
sudo apt update
sudo apt install nvidia-jetpack
sudo apt show nvidia-jetpack

## Upgrade compute stack
sudo apt-get install nvidia-jetpack

## Set GPU and CPU clocks to max
sudo /usr/bin/jetson_clocks

## Set power to max settings
sudo /usr/sbin/nvpmodel -m 0

## Add user to DOCKER group
sudo usermod -aG docker $USER
newgrp docker

## Setup docker

## Install latest docker version
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

## Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

## Add the repository to Apt sources:
echo \
"deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
"$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

sudo apt install docker-buildx-plugin

## Developer environment setup

## Restart docker
sudo systemctl daemon-reload && sudo systemctl restart docker

## Install Git LFS
sudo apt-get install git-lfs
git lfs install --skip-repo

## Setting up Isaac Apt Repo

## Set locale
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

## Install dependencies
sudo apt update && sudo apt install gnupg wget
sudo apt install software-properties-common
sudo add-apt-repository universe

## Setup source
wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo apt-key add -
grep -qxF "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" /etc/apt/sources.list || \
echo "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" | sudo tee -a /etc/apt/sources.list
sudo apt-get update

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

## Install core ROS2 packages
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools

## Resolve OCI error from solution at: https://forums.developer.nvidia.com/t/isaac-ros-setup-docker-launch-failed/316715 (before trying out the examples)
sudo nvidia-ctk cdi generate --mode=csv --output=/etc/cdi/nvidia.yaml

sudo apt-get update
sudo apt-get install software-properties-common
sudo apt-key adv --fetch-key https://repo.download.nvidia.com/jetson/jetson-ota-public.asc
sudo add-apt-repository 'deb https://repo.download.nvidia.com/jetson/common r36.4 main'
sudo apt-get update
sudo apt-get install -y pva-allow-2

## Create ROS2 workspace for Isaac ROS
mkdir -p  ~/workspaces/isaac_ros-dev/src
echo "export ISAAC_ROS_WS=~/workspaces/isaac_ros-dev/" >> ~/.bashrc
source ~/.bashrc

sudo reboot now