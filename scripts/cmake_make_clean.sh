#!/bin/bash
# Builds squirrel defender after cleaning build folder
#
# note: intended to be run inside the dev container

# Navigate to the OS_WS directory
cd "$OS_WS" || { echo "Failed to navigate to $OS_WS"; exit 1; }

# Navigate to the SquirrelDefender build directory
cd ./SquirrelDefender/build || { echo "Failed to navigate to SquirrelDefender/build"; exit 1; }

# Remove everything except files containing "engine" in their name
find . -type f ! -name '*engine*' -delete
find . -type d -empty -delete

# Run cmake and make
cmake ..
make -j"$(nproc)"