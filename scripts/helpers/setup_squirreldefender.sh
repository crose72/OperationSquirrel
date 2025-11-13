#!/bin/bash

# Default display
DISPLAY_NUM="0"

# -- Required Argument Parsing --
JETSON_TYPE=""
for arg in "$@"; do
    if [[ $arg == --jetson=* ]]; then
        JETSON_TYPE="${arg#*=}"
    fi
done

# -- Validate Argument --
if [[ "$JETSON_TYPE" != "orin" && "$JETSON_TYPE" != "b01" ]]; then
    echo "❌ Error: You must specify --jetson=orin or --jetson=b01"
    exit 1
fi

# Parse --display=1
for arg in "$@"; do
    if [[ $arg == --display=* ]]; then
        DISPLAY_NUM="${arg#*=}"
    fi
done

# Set script variables
USER_NAME=$(whoami)
WORKSPACE_PATH="/home/$USER_NAME/workspaces/os-dev"
XPROFILE_PATH="/home/$USER_NAME/.xprofile"
BASHRC_PATH="/home/$USER_NAME/.bashrc"
CLOCK_FIX_SCRIPT="/usr/local/bin/clock-skew-fix.sh"
SQUIRRELDEFENDER_SERVICE="/etc/systemd/system/squirreldefender.service"
CLOCK_FIX_SERVICE="/etc/systemd/system/clock-skew-fix.service"
BUILD_DIR=".$SCRIPT_DIR/../SquirrelDefender/build"

# 1. Add user to docker
# Note: commented out, doesn't work in script apparently
# sudo usermod -aG docker $USER && newgrp docker

# 2. Create or update .xprofile (allows the containers access to the camera and display)
echo "Creating ~/.xprofile..."
cat <<EOF > "$XPROFILE_PATH"
export DISPLAY=:0
xhost +
EOF
chmod +x "$XPROFILE_PATH"

# 3. Update .bashrc (set environment variables used by the containers)

grep -qxF "export DISPLAY=:$DISPLAY_NUM" "$BASHRC_PATH" || echo "export DISPLAY=:$DISPLAY_NUM" >> "$BASHRC_PATH"
grep -qxF "export OS_WS=$WORKSPACE_PATH" "$BASHRC_PATH" || echo "export OS_WS=$WORKSPACE_PATH" >> "$BASHRC_PATH"

echo "Appended to .bashrc:"
echo "  export DISPLAY=:$DISPLAY_NUM"
echo "  export OS_WS=$WORKSPACE_PATH"

# 4. Create squirreldefender.service (runs the squirreldefender container)
echo "Creating squirreldefender.service for Jetson: $JETSON_TYPE"
cat <<EOF | sudo tee "$SQUIRRELDEFENDER_SERVICE" > /dev/null
[Unit]
Description=SquirrelDefender program
After=network.target nvargus-daemon.service graphical.target multi-user.target
Wants=graphical.target

[Service]
Environment="OS_WS=$WORKSPACE_PATH"
Restart=off
ExecStartPre=/bin/bash -c 'sleep 5'
ExecStart=/bin/bash $WORKSPACE_PATH/OperationSquirrel/scripts/run.sh squirreldefender ${JETSON_TYPE}
ExecStop=/usr/bin/docker stop squirreldefender
StandardOutput=journal
StandardError=journal
User=root

[Install]
WantedBy=multi-user.target
EOF

# 5. Create the clock skew fix script (makes it possible to compile code when the jetson is not conneceted to wifi)
echo "Creating clock-skew-fix.sh..."
cat <<EOF | sudo tee "$CLOCK_FIX_SCRIPT" > /dev/null
#!/bin/bash

latest=\$(find /home/$USER_NAME/workspaces/os-dev/OperationSquirrel/SquirrelDefender -type f -exec stat -c %Y {} + 2>/dev/null | sort -n | tail -1)
now=\$(date +%s)

if [ -n "\$latest" ] && [ "\$now" -lt "\$latest" ]; then
    echo "Clock behind file timestamps, setting time forward..."
    date -s "@\$((\$latest + 10))"
fi
EOF
sudo chmod +x "$CLOCK_FIX_SCRIPT"

# 6. Create the clock-skew-fix.service (run the clock skew fix script)
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

# 7. Create build folder and copy engine file ()
echo "Ensuring build directory exists at $BUILD_DIR..."
mkdir -p "$BUILD_DIR"

# 8. Enable the systemd services
# sudo systemctl enable squirreldefender.service # allow users to enable the program when they want
sudo systemctl enable clock-skew-fix.service

# 9. Configure the camera
sudo /opt/nvidia/jetson-io/jetson-io.py

# 10. Final steps
echo "To complete the setup: 
    source ~/.bashrc
    chmod +x ~/.xprofile
    sudo reboot now"
