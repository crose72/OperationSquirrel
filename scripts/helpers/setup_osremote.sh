#!/usr/bin/env bash
set -e  # Exit immediately on error

USER_NAME=$(whoami)
SCRIPT_PATH="/home/$USER_NAME/workspaces/os-dev/OperationSquirrel/scripts/toggle_osremote.sh"
DATA_PATH="/home/$USER_NAME/workspaces/os-dev/OperationSquirrel/SquirrelDefender/data/*"
SUDOERS_FILE="/etc/sudoers.d/osremote_toggle"
HOTSPOT_NAME="$(hostname | tr '[:lower:]' '[:upper:]')_AP"
HOTSPOT_PASSWORD="SquirrelDefender"

# Detect Wi-Fi interface automatically
WIFI_IF=$(nmcli device | awk '/wifi/ {print $1; exit}')
if [ -z "$WIFI_IF" ]; then
  echo "❌ No Wi-Fi interface detected. Aborting."
  exit 1
fi

echo "------------------------------------------------------------"
echo "🐿️  Running OSRemote setup for user: $USER_NAME"
echo "Detected Wi-Fi interface: $WIFI_IF"
echo "------------------------------------------------------------"

# --------------------------------------------------------------------------
# 🔐 Allow passwordless sudo for key OSRemote commands
# --------------------------------------------------------------------------
{
  echo "$USER_NAME ALL=(ALL) NOPASSWD: $SCRIPT_PATH"
  echo "$USER_NAME ALL=(ALL) NOPASSWD: /bin/rm -rf $DATA_PATH"
} | sudo tee "$SUDOERS_FILE" > /dev/null
sudo chmod 440 "$SUDOERS_FILE"
echo "✅ Passwordless sudo rules updated."

# --------------------------------------------------------------------------
# 🧩 Allow NetworkManager control without prompts (for nmcli)
# --------------------------------------------------------------------------
sudo tee /etc/polkit-1/localauthority/50-local.d/50-networkmanager.pkla >/dev/null <<'EOF'
[Allow nmcli for all users]
Identity=unix-user:*
Action=org.freedesktop.NetworkManager.*
ResultAny=yes
ResultInactive=yes
ResultActive=yes
EOF

sudo systemctl restart polkit
echo "✅ NetworkManager polkit permissions configured."

# --------------------------------------------------------------------------
# 🛜 Configure or recreate hotspot
# --------------------------------------------------------------------------
if nmcli connection show "$HOTSPOT_NAME" &>/dev/null; then
  echo "Updating existing hotspot '$HOTSPOT_NAME'..."
  sudo nmcli connection modify "$HOTSPOT_NAME" ifname "$WIFI_IF" \
    802-11-wireless.mode ap \
    802-11-wireless.band bg \
    802-11-wireless-security.key-mgmt wpa-psk \
    802-11-wireless-security.psk "$HOTSPOT_PASSWORD" \
    ipv4.method shared ipv6.method ignore autoconnect yes
else
  echo "Creating new hotspot '$HOTSPOT_NAME'..."
  sudo nmcli connection add type wifi ifname "$WIFI_IF" con-name "$HOTSPOT_NAME" \
    ssid "$HOTSPOT_NAME" mode ap \
    wifi.band bg wifi.channel 6 \
    802-11-wireless-security.key-mgmt wpa-psk \
    802-11-wireless-security.psk "$HOTSPOT_PASSWORD" \
    ipv4.method shared ipv6.method ignore autoconnect yes
fi

# Bring up hotspot immediately (safe if already active)
sudo nmcli connection up "$HOTSPOT_NAME" || echo "⚠️ Could not start hotspot now; will start on next boot."

# --------------------------------------------------------------------------
# 🧠 Guarantee hotspot starts on boot (systemd service)
# --------------------------------------------------------------------------
SERVICE_FILE="/etc/systemd/system/oss_ap-autostart.service"
echo "Creating systemd autostart service for $HOTSPOT_NAME..."
sudo tee "$SERVICE_FILE" >/dev/null <<EOF
[Unit]
Description=Auto-start $HOTSPOT_NAME hotspot
After=NetworkManager.service
Wants=NetworkManager.service
ConditionPathExists=/sys/class/net/$WIFI_IF

[Service]
Type=oneshot
ExecStart=/usr/bin/nmcli connection up $HOTSPOT_NAME
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable oss_ap-autostart.service
echo "✅ Enabled systemd autostart for $HOTSPOT_NAME"

# --------------------------------------------------------------------------
# 🧩 Finish
# --------------------------------------------------------------------------
mkdir -p ~/.osremote_net

echo "✅ OSRemote network configuration complete."
echo "------------------------------------------------------------"
echo "   🛜 Hotspot: $HOTSPOT_NAME"
echo "   🔑 Password: $HOTSPOT_PASSWORD"
echo "   📡 Interface: $WIFI_IF"
echo "   🚀 Autostart service: oss_ap-autostart.service (enabled)"
echo "------------------------------------------------------------"
echo "Setup complete. You can reboot now — the hotspot will auto-start."
