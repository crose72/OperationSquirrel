#!/bin/bash
set -e

SETUP_TARGET="${1}"           # squirreldefender | osremote
JETSON_FLAG="${2:-}"          # optional flag (for squirreldefender)
SCRIPT_DIR="$(dirname "$0")"
HELPER_DIR="$SCRIPT_DIR/helpers"

USER_NAME=$(whoami)
WORKSPACE_PATH="/home/$USER_NAME/workspaces/os-dev"
BASHRC_PATH="/home/$USER_NAME/.bashrc"
XPROFILE_PATH="/home/$USER_NAME/.xprofile"

# --------------------------------------------------------------------------
# Common environment setup (for both squirreldefender and OSRemote)
# --------------------------------------------------------------------------
echo "🧩 Running common setup for all operationsquirrel tools..."

# 1. Ensure workspace exists
mkdir -p "$WORKSPACE_PATH"
echo "✅ Workspace ensured at: $WORKSPACE_PATH"

# 2. Create .xprofile if missing (for display and camera)
if [[ ! -f "$XPROFILE_PATH" ]]; then
    echo "Creating ~/.xprofile..."
    cat <<EOF > "$XPROFILE_PATH"
export DISPLAY=:0
xhost +
EOF
    chmod +x "$XPROFILE_PATH"
    echo "✅ Created .xprofile"
fi

# 3. Add environment variables to .bashrc if not already present
if ! grep -q "export DISPLAY=" "$BASHRC_PATH"; then
    echo "export DISPLAY=:0" >> "$BASHRC_PATH"
    echo "✅ Added DISPLAY to .bashrc"
fi

if ! grep -q "export OS_WS=" "$BASHRC_PATH"; then
    echo "export OS_WS=$WORKSPACE_PATH" >> "$BASHRC_PATH"
    echo "✅ Added OS_WS to .bashrc"
fi

# --------------------------------------------------------------------------
# Select and run helper
# --------------------------------------------------------------------------
if [[ "$SETUP_TARGET" == "squirreldefender" ]]; then
    HELPER_SCRIPT="$HELPER_DIR/setup_squirreldefender.sh"
elif [[ "$SETUP_TARGET" == "osremote" ]]; then
    HELPER_SCRIPT="$HELPER_DIR/setup_osremote.sh"
else
    echo "❌ Invalid setup target: $SETUP_TARGET"
    echo "Usage:"
    echo "  ./setup.sh squirreldefender --jetson=[orin|b01]"
    echo "  ./setup.sh osremote"
    exit 1
fi

if [[ ! -f "$HELPER_SCRIPT" ]]; then
    echo "❌ Missing helper script: $HELPER_SCRIPT"
    exit 1
fi

echo "🚀 Running helper setup: $HELPER_SCRIPT $JETSON_FLAG"
bash "$HELPER_SCRIPT" "$JETSON_FLAG"

echo "✅ Setup complete for $SETUP_TARGET."
echo "   (You may need to 'source ~/.bashrc' or reboot for environment updates.)"

# After setup
source ~/.bashrc