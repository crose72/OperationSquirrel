#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

"$SCRIPT_DIR/01_detect_wifi.sh"
"$SCRIPT_DIR/02_setup_sudo_rules.sh"
"$SCRIPT_DIR/03_setup_nm_permissions.sh"
"$SCRIPT_DIR/04_setup_hotspot.sh"
"$SCRIPT_DIR/05_setup_autostart.sh"

echo "------------------------------------------------------------"
echo "🎉 OSRemote Setup Complete"
echo "   Hotspot: $HOTSPOT_NAME"
echo "   Password: $HOTSPOT_PASSWORD"
echo "   WiFi Interface: $WIFI_IF"
echo "------------------------------------------------------------"
echo "You may reboot now to enable autostart:"
echo "  sudo reboot"
