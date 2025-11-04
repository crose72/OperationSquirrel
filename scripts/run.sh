#!/bin/bash
set -e

CONTAINER="${1}"      # dev or squirreldefender
PLATFORM="${2}"            # orin, b01, ubuntu-22.04_sm86

SCRIPT_DIR="$(dirname "$0")"
HELPER_DIR="$SCRIPT_DIR/helpers"

# Validate CONTAINER
if [[ "$CONTAINER" != "dev" && "$CONTAINER" != "squirreldefender" ]]; then
    echo "❌ Invalid arguments"
    echo "Usage: ./run.sh [dev|squirreldefender] [orin|b01|ubuntu-22.04_sm86]"
    exit 1
fi

echo "🚀 Container: $CONTAINER"
echo "🧩 Platform: $PLATFORM"

# Map platform -> version identifier
case "$PLATFORM" in
    orin)
        RUN_TAG="r36.4.0"
        ;;
    b01)
        RUN_TAG="r32.7.1"
        ;;
    ubuntu-22.04_sm86)
        RUN_TAG="ubuntu-22.04_sm86"
        ;;
    *)
        echo "❌ Invalid platform: $PLATFORM"
        echo "Usage: ./run.sh [dev|squirreldefender] [orin|b01|ubuntu-22.04_sm86]"
        exit 1
        ;;
esac

# Construct the helper name dynamically
HELPER_SCRIPT="$HELPER_DIR/run_${CONTAINER}_${RUN_TAG}.sh"

if [[ ! -f "$HELPER_SCRIPT" ]]; then
    echo "❌ Missing helper: $HELPER_SCRIPT"
    echo "Expected at: $HELPER_SCRIPT"
    exit 1
fi

echo "📦 Using version tag: $RUN_TAG"
shift 2
bash "$HELPER_SCRIPT" "$@"
