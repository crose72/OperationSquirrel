#!/bin/bash
set -e

CONTAINER="${1}"   # dev or squirreldefender
PLATFORM="${2}"    # orin, b01

SCRIPT_DIR="$(dirname "$0")"
HELPER_DIR="$SCRIPT_DIR/helpers"

# Validate CONTAINER
if [[ "$CONTAINER" != "dev" && "$CONTAINER" != "squirreldefender" ]]; then
    echo "❌ Invalid arguments"
    echo "Usage: ./build.sh [dev|squirreldefender] [orin|b01]"
    exit 1
fi

echo "🛠️  Build type: $CONTAINER"
echo "🧩 Platform:    $PLATFORM"

# Map platform -> version identifier
case "$PLATFORM" in
    orin)
        BUILD_TAG="r36.4.0"
        ;;
    b01)
        BUILD_TAG="r32.7.1"
        ;;
    ubuntu-22.04-sm86)
        BUILD_TAG="ubuntu-22.04-sm86"
        ;;
    *)
        echo "❌ Invalid platform: $PLATFORM"
        echo "Usage: ./build.sh [dev|squirreldefender] [orin|b01]"
        exit 1
        ;;
esac

# Construct the helper path dynamically
HELPER_SCRIPT="$HELPER_DIR/build_${CONTAINER}_${BUILD_TAG}.sh"

# Handle future dev builds gracefully
if [[ "$CONTAINER" == "dev" ]]; then
    echo "🧱 Dev build placeholder for $BUILD_TAG"
    echo "   (No helper script defined yet — add one later at: $HELPER_SCRIPT)"
    exit 0
fi

# Validate helper existence
if [[ ! -f "$HELPER_SCRIPT" ]]; then
    echo "❌ Missing helper: $HELPER_SCRIPT"
    echo "Expected at: $HELPER_SCRIPT"
    exit 1
fi

echo "📦 Using build tag: $BUILD_TAG"
shift 2
bash "$HELPER_SCRIPT" "$@"
