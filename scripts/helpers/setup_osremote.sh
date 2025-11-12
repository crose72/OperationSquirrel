#!/usr/bin/env bash
USER_NAME=$(whoami)
SCRIPT_PATH="/home/$USER_NAME/workspaces/os-dev/OperationSquirrel/scripts/toggle_osremote.sh"

# Allow passwordless toggle
echo "$USER_NAME ALL=(ALL) NOPASSWD: $SCRIPT_PATH" | sudo tee /etc/sudoers.d/osremote_toggle > /dev/null
sudo chmod 440 /etc/sudoers.d/osremote_toggle

# Allow nmcli control (optional safety)
sudo tee /etc/polkit-1/localauthority/50-local.d/50-networkmanager.pkla >/dev/null <<'EOF'
[Allow nmcli for all users]
Identity=unix-user:*
Action=org.freedesktop.NetworkManager.*
ResultAny=yes
ResultInactive=yes
ResultActive=yes
EOF

sudo systemctl restart polkit

mkdir -p ~/.osremote_net

echo "✅ OSRemote setup complete. You can now run:"
echo "   sudo $SCRIPT_PATH"
