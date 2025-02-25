#!/bin/bash

# Set variables (change these)
DOCKERFILE_PATH="../docker/Dockerfile.squirreldefender"       # Path to your Dockerfile
IMAGE_NAME="crose72/jetpack-r36.4.0"  # Docker image name
IMAGE_TAG="squirreldefender"                          # Tag for the Docker image
FULL_IMAGE_NAME="$IMAGE_NAME:$IMAGE_TAG"    # Full image name with tag

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
