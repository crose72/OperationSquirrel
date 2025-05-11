#!/bin/bash

# Set variables
USER_NAME=$(whoami)
WORKSPACE_PATH="/home/$USER_NAME/workspaces/os-dev"
XPROFILE_PATH="/home/$USER_NAME/.xprofile"
CLOCK_FIX_SCRIPT="/usr/local/bin/clock-skew-fix.sh"
SQUIRRELDEFENDER_SERVICE="/etc/systemd/system/squirreldefender.service"
CLOCK_FIX_SERVICE="/etc/systemd/system/clock-skew-fix.service"
BUILD_DIR="$SCRIPT_DIR/../SquirrelDefender/build"
ENGINE_FILE="$SCRIPT_DIR/../SquirrelDefender/networks/yolov8s/yolov8s.engine.Orin.fp16.1.1.-1.-1.-1"

# 1. Create or update .xprofile
echo "Creating ~/.xprofile..."
cat <<EOF > "$XPROFILE_PATH"
export DISPLAY=:0
xhost +
EOF
chmod +x "$XPROFILE_PATH"

# 2. Create squirreldefender.service
echo "Creating squirreldefender.service..."
cat <<EOF | sudo tee "$SQUIRRELDEFENDER_SERVICE" > /dev/null
[Unit]
Description=SquirrelDefender program
After=network.target nvargus-daemon.service graphical.target multi-user.target
Wants=graphical.target

[Service]
Environment="OS_WS=$WORKSPACE_PATH"
Restart=off
ExecStartPre=/bin/bash -c 'sleep 5'
ExecStart=/bin/bash $WORKSPACE_PATH/OperationSquirrel/scripts/run_squirreldefender_orin.sh
ExecStop=/usr/bin/docker stop squirreldefender
StandardOutput=journal
StandardError=journal
User=root

[Install]
WantedBy=multi-user.target
EOF

# 3. Create the clock skew fix script
echo "Creating clock-skew-fix.sh..."
cat <<'EOF' | sudo tee "$CLOCK_FIX_SCRIPT" > /dev/null
#!/bin/bash

latest=$(find /home/crose72/workspaces/os-dev/OperationSquirrel/SquirrelDefender -type f -exec stat -c %Y {} + 2>/dev/null | sort -n | tail -1)
now=$(date +%s)

if [ -n "$latest" ] && [ "$now" -lt "$latest" ]; then
    echo "Clock behind file timestamps, setting time forward..."
    date -s "@$((latest + 10))"
fi
EOF
sudo chmod +x "$CLOCK_FIX_SCRIPT"

# 4. Create the clock-skew-fix.service
echo "Creating clock-skew-fix.service..."
cat <<EOF | sudo tee "$CLOCK_FIX_SERVICE" > /dev/null
[Unit]
Description=Ensure system clock is ahead of all file timestamps in the project
After=multi-user.target

[Service]
Type=oneshot
ExecStart=$CLOCK_FIX_SCRIPT

[Install]
WantedBy=multi-user.target
EOF

# 5. Create build folder and copy engine file
echo "Ensuring build directory exists at $BUILD_DIR..."
mkdir -p "$BUILD_DIR"

echo "Copying engine file to build directory..."
if [ -f "$ENGINE_FILE" ]; then
    cp "$ENGINE_FILE" "$BUILD_DIR/"
    echo "Copied: $(basename "$ENGINE_FILE")"
else
    echo "Warning: Engine file not found at $ENGINE_FILE"
fi

# 6 Configure the camera
sudo /opt/nvidia/jetson-io/jetson-io.py

# Done
echo "All files created. To enable services, run:"
echo "  sudo systemctl enable clock-skew-fix.service"
echo "  sudo systemctl enable squirreldefender.service"
