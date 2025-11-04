#!/bin/bash

# Get memory information
memory_info=$(free -m)

# Extract total memory using awk
mem=$(echo "$memory_info" | awk 'NR==2 {print $2}')

# Extract swap information using awk
swap=$(echo "$memory_info" | awk '/Swap:/ {print $2}')

# Perform addition
total_memory=$((mem + swap))

# Display the total memory and swap
echo "Total Memory: $total_memory MB"

if [ "$total_memory" -lt 12288 ]; then
	echo "Not enough memory, need at least 12288 MB, you have $total_memory MB"
	exit 1
else
	echo "Yay! You have enough memory for ORB SLAM3, proceed."
fi
