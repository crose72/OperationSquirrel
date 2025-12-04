#!/bin/bash
set -e

WORKSPACE_PATH="$OS_WS"
USER_NAME=$(whoami)

CLOCK_FIX_SCRIPT="/usr/local/bin/clock-skew-fix.sh"
CLOCK_FIX_SERVICE="/etc/systemd/system/clock-skew-fix.service"
SQUIRRELDEFENDER_SERVICE="/etc/systemd/system/squirreldefender.service"

echo "Creating squirreldefender.service..."
cat <<EOF | sudo tee "$SQUIRRELDEFENDER_SERVICE" > /dev/null
[Unit]
Description=squirreldefender program
After=network.target nvargus-daemon.service graphical.target multi-user.target
Wants=graphical.target

[Service]
Environment="OS_WS=$WORKSPACE_PATH"
Restart=off
ExecStartPre=/bin/bash -c 'sleep 5'
ExecStart=/bin/bash "$WORKSPACE_PATH/operationsquirrel/scripts/run.sh" squirreldefender $JETSON_TYPE
ExecStop=/usr/bin/docker stop squirreldefender
StandardOutput=journal
StandardError=journal
User=root

[Install]
WantedBy=multi-user.target
EOF

echo "Creating clock-skew-fix.sh..."
cat <<EOF | sudo tee "$CLOCK_FIX_SCRIPT" > /dev/null
#!/bin/bash
latest=\$(find /home/$USER_NAME/workspaces/os-dev/operationsquirrel/squirreldefender -type f -exec stat -c %Y {} + 2>/dev/null | sort -n | tail -1)
now=\$(date +%s)
if [ -n "\$latest" ] && [ "\$now" -lt "\$latest" ]; then
    date -s "@\$((\$latest + 10))"
fi
EOF

sudo chmod +x "$CLOCK_FIX_SCRIPT"

echo "Creating clock-skew-fix.service..."
cat <<EOF | sudo tee "$CLOCK_FIX_SERVICE" > /dev/null
[Unit]
Description=Fix system clock skew
After=multi-user.target

[Service]
Type=oneshot
ExecStart=$CLOCK_FIX_SCRIPT

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl enable clock-skew-fix.service
