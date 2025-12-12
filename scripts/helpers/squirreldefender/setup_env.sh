#!/bin/bash
set -e

# --------------------------------------------------------------------------
# Parse args
# --------------------------------------------------------------------------
JETSON_TYPE=""
DISPLAY_NUM="0"

for arg in "$@"; do
    case $arg in
        --jetson=*)
            JETSON_TYPE="${arg#*=}"
            ;;
        --display=*)
            DISPLAY_NUM="${arg#*=}"
            ;;
    esac
done

if [[ "$JETSON_TYPE" != "orin" && "$JETSON_TYPE" != "b01" ]]; then
    echo "❌ Error: You must specify --jetson=orin or --jetson=b01"
    exit 1
fi

export JETSON_TYPE DISPLAY_NUM

USER_NAME=$(whoami)
export USER_NAME
WORKSPACE_PATH="/home/$USER_NAME/workspaces/os-dev"
export WORKSPACE_PATH

XPROFILE_PATH="/home/$USER_NAME/.xprofile"
BASHRC_PATH="/home/$USER_NAME/.bashrc"

# --------------------------------------------------------------------------
# Fix or create ~/.xprofile with correct ownership
# --------------------------------------------------------------------------
# If ~/.xprofile exists but is not owned by the user -> fix it
if [[ -e "$XPROFILE_PATH" ]]; then
    OWNER=$(stat -c %U "$XPROFILE_PATH")
    if [[ "$OWNER" != "$USER_NAME" ]]; then
        echo "⚠️ ~/.xprofile owned by $OWNER — correcting ownership..."
        sudo chown "$USER_NAME:$USER_NAME" "$XPROFILE_PATH"
    fi
else
    echo "Creating ~/.xprofile..."
fi

# Write the updated DISPLAY settings
cat <<EOF > "$XPROFILE_PATH"
export DISPLAY=:$DISPLAY_NUM
xhost +
EOF

chmod +x "$XPROFILE_PATH"
echo "✅ ~/.xprofile updated with DISPLAY=:$DISPLAY_NUM"

# --------------------------------------------------------------------------
# Update ~/.bashrc with DISPLAY and OS_WS
# --------------------------------------------------------------------------

# Replace existing DISPLAY line or add it if missing
if grep -q "^export DISPLAY=" "$BASHRC_PATH"; then
    sed -i "s/^export DISPLAY=.*/export DISPLAY=:$DISPLAY_NUM/" "$BASHRC_PATH"
else
    echo "export DISPLAY=:$DISPLAY_NUM" >> "$BASHRC_PATH"
fi
echo "✅ ~/.bashrc DISPLAY updated to :$DISPLAY_NUM"

# Add OS_WS only if missing
if ! grep -q "export OS_WS=" "$BASHRC_PATH"; then
    echo "export OS_WS=$WORKSPACE_PATH" >> "$BASHRC_PATH"
    echo "✅ Added OS_WS to ~/.bashrc"
fi

# --------------------------------------------------------------------------
# Ensure build folder structure exists for squirreldefender
# --------------------------------------------------------------------------
SDF_PATH="$WORKSPACE_PATH/operationsquirrel/squirreldefender"
BUILD_DIR="$SDF_PATH/build"
CMAKE_DIR="$BUILD_DIR/.cmake"

echo "📁 Ensuring squirreldefender build directories exist..."

if [[ ! -d "$BUILD_DIR" ]]; then
    mkdir -p "$BUILD_DIR"
    echo "  ➕ Created: $BUILD_DIR"
fi

if [[ ! -d "$CMAKE_DIR" ]]; then
    mkdir -p "$CMAKE_DIR"
    echo "  ➕ Created: $CMAKE_DIR"
fi

echo "✅ Build directory ready: $CMAKE_DIR"
