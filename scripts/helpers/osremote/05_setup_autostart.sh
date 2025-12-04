#!/usr/bin/env bash
set -e

# Validate required vars
if [[ -z "$HOTSPOT_NAME" || -z "$WIFI_IF" ]]; then
    echo "❌ Missing autostart environment variables."
    exit 1
fi

SERVICE_FILE="/etc/systemd/system/oss_ap-autostart.service"

echo "⚙️ Creating systemd autostart service..."

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

mkdir -p ~/.osremote_net

echo "✅ Hotspot autostart enabled."
