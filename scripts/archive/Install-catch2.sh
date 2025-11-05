#!/bin/bash

# -----------------------------------------------
# Install ORB_SLAM3 dependencies
# -----------------------------------------------

echo "Installing catch2"

git clone https://github.com/catchorg/Catch2.git
cd Catch2
cmake -Bbuild -H. -DBUILD_TESTING=OFF
sudo cmake --build build/ --target install 
