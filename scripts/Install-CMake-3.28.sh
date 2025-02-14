#!/bin/bash

# Remove old version of cmake and install a newer one
sudo apt-get remove cmake  # or your package manager's equivalent command

# Replace <version> with the version number you downloaded
wget https://cmake.org/files/v3.28/cmake-3.28.0-linux-aarch64.sh
chmod +x cmake-3.28.0-linux-aarch64.sh
sudo ./cmake-3.28.0-linux-aarch64.sh --prefix=/usr/local --exclude-subdir
