#!/usr/bin/env bash
set -e

USER_NAME=$(whoami)
export USER_NAME

SCRIPT_PATH="/home/$USER_NAME/workspaces/os-dev/operationsquirrel/scripts/toggle_osremote.sh"
DATA_PATH="/home/$USER_NAME/workspaces/os-dev/operationsquirrel/squirreldefender/data/*"
SUDOERS_FILE="/etc/sudoers.d/osremote_toggle"
HOTSPOT_NAME="$(hostname | tr '[:lower:]' '[:upper:]')_AP"
HOTSPOT_PASSWORD="squirreldefender"

export SCRIPT_PATH DATA_PATH SUDOERS_FILE HOTSPOT_NAME HOTSPOT_PASSWORD

# Detect Wi-Fi interface
WIFI_IF=$(nmcli device | awk '/wifi/ {print $1; exit}')
if [ -z "$WIFI_IF" ]; then
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
