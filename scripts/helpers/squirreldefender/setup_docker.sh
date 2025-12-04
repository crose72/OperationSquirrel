#!/bin/bash
set -e

echo "➡ Installing Docker for JetPack 6.x (Jetson-safe)…"

# 0. Remove upstream Docker repo (prevents containerd conflicts)
if [ -f /etc/apt/sources.list.d/docker.list ]; then
    echo "⚠️ Removing upstream Docker repo to prevent conflicts..."
    sudo rm /etc/apt/sources.list.d/docker.list
fi

# 1. Update package lists + install curl/gnupg2
sudo apt-get update
sudo apt-get install -y curl gnupg2

# 2. Install only docker.io if not already installed
#    (JetPack's NVIDIA containerd works ONLY with docker.io)
if ! command -v docker &> /dev/null; then
    echo "📦 Docker not found — installing docker.io..."
    sudo apt-get install -y docker.io
    sudo systemctl enable docker
    sudo systemctl start docker
else
    echo "✔ Docker already installed — skipping."
fi
