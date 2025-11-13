#!/usr/bin/env bash
set -e

# ------------------------------------------------------------------
# 🦊 OSRemote Wi-Fi / Hotspot Toggle Script (Jetson Edition)
# ------------------------------------------------------------------

# Force correct home directory even under sudo
if [ -n "$SUDO_USER" ]; then
    USER_HOME=$(getent passwd "$SUDO_USER" | cut -d: -f6)
else
    USER_HOME="$HOME"
fi

HOTSPOT_NAME="OSS_AP"
HOTSPOT_PASS="SquirrelDefender"
WIFI_DEV="wlP1p1s0"
CONFIG_DIR="$USER_HOME/.osremote_net"
STATE_FILE="$CONFIG_DIR/state"
WIFI_INFO_FILE="$CONFIG_DIR/wifi_info"

mkdir -p "$CONFIG_DIR"

# --- Ensure network-manager exists ---
if ! command -v nmcli &>/dev/null; then
    echo "Installing network-manager..."
    sudo apt update && sudo apt install -y network-manager
fi

# --- Helper: reconnect to saved Wi-Fi ---
reconnect_wifi() {
    if [[ -f "$WIFI_INFO_FILE" ]]; then
        SSID=$(grep '^SSID=' "$WIFI_INFO_FILE" | cut -d= -f2-)
        PASS=$(grep '^PASS=' "$WIFI_INFO_FILE" | cut -d= -f2-)
        echo "🔁 Reconnecting to saved Wi-Fi: $SSID ..."

        # Force clean reset of the Wi-Fi interface
        sudo nmcli dev disconnect "$WIFI_DEV" || true
        sudo nmcli radio wifi off
        sleep 2
        sudo nmcli radio wifi on

        # ✅ Wait until the interface is 'available'
        echo -n "⏳ Waiting for $WIFI_DEV to become available"
        for i in {1..10}; do
            STATE=$(nmcli -t -f DEVICE,STATE dev | grep "^${WIFI_DEV}:" | cut -d: -f2)
            if [[ "$STATE" == "disconnected" || "$STATE" == "unavailable" ]]; then
                echo -n "."
                sleep 1
            else
                break
            fi
        done
        echo

        # If still unavailable, bail
        STATE=$(nmcli -t -f DEVICE,STATE dev | grep "^${WIFI_DEV}:" | cut -d: -f2)
        if [[ "$STATE" == "unavailable" ]]; then
            echo "⚠️ $WIFI_DEV still unavailable — aborting reconnect."
            return 1
        fi

        echo "📡 Scanning for networks..."
        nmcli dev wifi rescan
        sleep 2

        echo "🔗 Connecting to $SSID..."
        sudo nmcli dev wifi connect "$SSID" password "$PASS" ifname "$WIFI_DEV" || {
            echo "⚠️ Failed to reconnect to $SSID."
            echo "ℹ️ Try manually: sudo nmcli dev wifi connect \"$SSID\" password \"$PASS\""
            return 1
        }

        echo "✅ Connected to $SSID!"
    else
        echo "⚠️ No saved Wi-Fi credentials found at $WIFI_INFO_FILE"
    fi
}

# --- Detect current Wi-Fi mode (AP vs managed) ---
CURRENT_MODE=$(iw dev "$WIFI_DEV" info 2>/dev/null | awk '/type/ {print $2}')
ACTIVE_CONN=$(nmcli -t -f NAME,DEVICE con show --active | grep "$WIFI_DEV" | cut -d: -f1)

# ------------------------------------------------------------------
# Case 1: Currently in Access Point (Hotspot) mode → switch to Wi-Fi
# ------------------------------------------------------------------
if [[ "$CURRENT_MODE" == "AP" || "$ACTIVE_CONN" == "$HOTSPOT_NAME" ]]; then
    echo "🟢 Currently in hotspot mode — switching back to normal Wi-Fi..."
    echo "mode=wifi" > "$STATE_FILE"
    reconnect_wifi
    exit 0
fi

# ------------------------------------------------------------------
# Case 2: Connected to Wi-Fi → switch to Hotspot
# ------------------------------------------------------------------
if [[ "$CURRENT_MODE" == "managed" && -n "$ACTIVE_CONN" ]]; then
    echo "📶 Connected to Wi-Fi: $ACTIVE_CONN"
    echo "mode=ap" > "$STATE_FILE"

    # Save Wi-Fi credentials (first time only)
    if [[ ! -f "$WIFI_INFO_FILE" ]]; then
        echo "Saving Wi-Fi credentials..."
        read -rp "Enter Wi-Fi password for $ACTIVE_CONN: " -s PASS
        echo
        echo "SSID=$ACTIVE_CONN" > "$WIFI_INFO_FILE"
        echo "PASS=$PASS" >> "$WIFI_INFO_FILE"
        chmod 600 "$WIFI_INFO_FILE"
    fi

    echo "📡 Switching to Access Point mode..."
    sudo nmcli dev disconnect "$WIFI_DEV"
    sleep 1
    if ! nmcli con show "$HOTSPOT_NAME" &>/dev/null; then
        echo "Creating hotspot profile..."
        sudo nmcli dev wifi hotspot ifname "$WIFI_DEV" ssid "$HOTSPOT_NAME" password "$HOTSPOT_PASS"
    else
        echo "Starting existing hotspot..."
        sudo nmcli con up "$HOTSPOT_NAME"
    fi

    IP=$(nmcli -t -f IP4.ADDRESS dev show "$WIFI_DEV" | grep -oE '([0-9]{1,3}\.){3}[0-9]{1,3}' | head -n1)
    echo "✅ Hotspot active: SSID=$HOTSPOT_NAME  PASS=$HOTSPOT_PASS  IP=${IP:-10.42.0.1}"
    exit 0
fi

# ------------------------------------------------------------------
# Case 3: No Wi-Fi detected (booted offline) → restore last known mode
# ------------------------------------------------------------------
if [[ -f "$STATE_FILE" && $(grep 'mode=ap' "$STATE_FILE") ]]; then
    echo "📡 Starting hotspot (last mode was AP)..."
    sudo nmcli dev wifi hotspot ifname "$WIFI_DEV" ssid "$HOTSPOT_NAME" password "$HOTSPOT_PASS"
else
    echo "🔁 Attempting to reconnect saved Wi-Fi..."
    reconnect_wifi
fi
