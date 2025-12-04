#!/usr/bin/env bash
set -e

# ------------------------------------------------------------
# OSRemote – Main Setup Script
# ------------------------------------------------------------

USER_NAME=$(whoami)
export USER_NAME

WORKSPACE="/home/$USER_NAME/workspaces/os-dev/operationsquirrel"
SCRIPT_PATH="$WORKSPACE/scripts/toggle_osremote.sh"
DATA_PATH="$WORKSPACE/squirreldefender/data/*"
SUDOERS_FILE="/etc/sudoers.d/osremote_toggle"
HOTSPOT_NAME="$(hostname | tr '[:lower:]' '[:upper:]')_AP"
HOTSPOT_PASSWORD="squirreldefender"

export SCRIPT_PATH DATA_PATH SUDOERS_FILE HOTSPOT_NAME HOTSPOT_PASSWORD

# ------------------------------------------------------------
# Detect Wi-Fi Interface
# ------------------------------------------------------------
WIFI_IF=$(nmcli device | awk '/wifi/ {print $1; exit}')
if [[ -z "$WIFI_IF" ]]; then
  echo "❌ No Wi-Fi interface detected. Aborting."
  exit 1
fi
export WIFI_IF

echo "------------------------------------------------------------"
echo "🐿️  OSRemote Setup"
echo "User: $USER_NAME"
echo "Detected Wi-Fi Interface: $WIFI_IF"
echo "Hotspot Name: $HOTSPOT_NAME"
echo "------------------------------------------------------------"

# ------------------------------------------------------------
# Run Stage Scripts
# ------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

"$SCRIPT_DIR/01_detect_wifi.sh"
"$SCRIPT_DIR/02_setup_sudo_rules.sh"
"$SCRIPT_DIR/03_setup_nm_permissions.sh"
"$SCRIPT_DIR/04_setup_hotspot.sh"
"$SCRIPT_DIR/05_setup_autostart.sh"

echo "------------------------------------------------------------"
echo "🎉 OSRemote Setup Complete"
echo "🛜 Hotspot:     $HOTSPOT_NAME"
echo "🔑 Password:    $HOTSPOT_PASSWORD"
echo "📡 Interface:   $WIFI_IF"
echo "------------------------------------------------------------"
echo "You may reboot now to enable autostart:"
echo "  sudo reboot"
