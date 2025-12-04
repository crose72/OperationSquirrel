#!/bin/bash
set -e

# Parse args
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

echo "Creating ~/.xprofile..."
cat <<EOF > /home/$USER_NAME/.xprofile
export DISPLAY=:$DISPLAY_NUM
xhost +
EOF

chmod +x /home/$USER_NAME/.xprofile

grep -qxF "export DISPLAY=:$DISPLAY_NUM" ~/.bashrc || echo "export DISPLAY=:$DISPLAY_NUM" >> ~/.bashrc
grep -qxF "export OS_WS=$WORKSPACE_PATH" ~/.bashrc || echo "export OS_WS=$WORKSPACE_PATH" >> ~/.bashrc
