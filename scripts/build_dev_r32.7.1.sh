#!/bin/bash

# Docker container build settings
DOCKERFILE_PATH="../docker/Dockerfile.dev_jetson-inference_r32.7.1"
IMAGE_NAME="crose72/jetson-inference-r32.7.1"
IMAGE_TAG="dev-latest"
FULL_IMAGE_NAME="$IMAGE_NAME:$IMAGE_TAG"

# Build Docker image
echo "🛠️ Building Docker image..."
docker build -f "$DOCKERFILE_PATH" -t "$FULL_IMAGE_NAME" ..

# Check if the build succeeded
if [ $? -eq 0 ]; then
    echo "✅ Docker image '$FULL_IMAGE_NAME' built successfully."
else
    echo "❌ Docker build failed!"
    exit 1
fi

# Optional: Login and push the image
read -p "Do you want to push the image to Docker Hub? [y/N] " push_response
if [[ "$push_response" == [yY]* ]]; then
    echo "🔑 Logging in to Docker Hub (if needed)..."
    docker login
    
    echo "🚀 Pushing Docker image '$FULL_IMAGE_NAME'..."
    docker push "$FULL_IMAGE_NAME"
fi
