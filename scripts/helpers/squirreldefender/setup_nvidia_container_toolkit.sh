#!/bin/bash
set -e

echo "📦 Installing NVIDIA Container Toolkit (JetPack repo)…"

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

echo "🔧 Configuring Docker to use NVIDIA runtime…"
sudo nvidia-ctk runtime configure --runtime=docker

sudo systemctl restart docker

echo "✔ NVIDIA Container Toolkit configured successfully."
