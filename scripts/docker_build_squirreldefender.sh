#!/bin/bash

# Set docker container build settings
DOCKERFILE_PATH="../docker/Dockerfile.squirreldefender"       # Path to your Dockerfile
IMAGE_NAME="crose72/jetpack-r36.4.0"  # Docker image name
IMAGE_TAG="squirreldefender"                          # Tag for the Docker image
FULL_IMAGE_NAME="$IMAGE_NAME:$IMAGE_TAG"    # Full image name with tag

# Source and destination paths for copying build outputs from the squirreldefender-field
CONTAINER_NAME="squirreldefender-field"
SRC_PATH="/workspace/OperationSquirrel/SquirrelDefender/build"
DEST_PATH="${OS_WS}/OperationSquirrel/SquirrelDefender/build-field"

# Set default build type to "dev"
BUILD_TYPE="dev"

# Check for argument to select "field" build
if [[ "$1" == "--field" ]]; then
    BUILD_TYPE="field"
fi

# Set the appropriate folder name based on build type
BUILD_FOLDER="build"  # Default for dev
if [ "$BUILD_TYPE" == "field" ]; then
    BUILD_FOLDER="build-field"
fi

# Make sure to define ${OS_WS}
if [ -z "$OS_WS" ]; then
    echo "❌ Error: OS_WS environment variable is not set."
    exit 1
fi

# Check if the field container exists
if docker ps -a --format '{{.Names}}' | grep -q "^$CONTAINER_NAME$"; then
    echo "📦 Container '$CONTAINER_NAME' found. Preparing destination directory..."

    # Remove the existing directory and create a fresh one
    if [ -d "$DEST_PATH" ]; then
        echo "🗑️ Clearing existing contents of $DEST_PATH..."
        rm -rf "$DEST_PATH"
    fi

    mkdir -p "$DEST_PATH"

    echo "📂 Copying files from container..."
    docker cp "$CONTAINER_NAME:$SRC_PATH/." "$DEST_PATH"


    echo "✅ Files copied successfully to $DEST_PATH."
else
    echo "⚠️ Container '$CONTAINER_NAME' not found. Skipping file copy."
fi

# Build Docker image
echo "🛠️ Building Docker image..."
docker build --build-arg BUILD_FOLDER="$BUILD_FOLDER" -f "$DOCKERFILE_PATH" -t "$FULL_IMAGE_NAME" ..

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
