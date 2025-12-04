#!/usr/bin/env bash
set -e

# Validate inputs
if [[ -z "$HOTSPOT_NAME" || -z "$WIFI_IF" || -z "$HOTSPOT_PASSWORD" ]]; then
    echo "❌ Missing hotspot environment variables."
    exit 1
fi

echo "📡 Configuring hotspot '$HOTSPOT_NAME'..."

# Update existing hotspot
if nmcli connection show "$HOTSPOT_NAME" &>/dev/null; then
  echo "🔄 Updating existing hotspot..."
  sudo nmcli connection modify "$HOTSPOT_NAME" ifname "$WIFI_IF" \
    802-11-wireless.mode ap \
    802-11-wireless.band bg \
    802-11-wireless-security.key-mgmt wpa-psk \
    802-11-wireless-security.psk "$HOTSPOT_PASSWORD" \
    ipv4.method shared ipv6.method ignore autoconnect yes

# Create new hotspot
else
  echo "➕ Creating hotspot '$HOTSPOT_NAME'..."
  sudo nmcli connection add type wifi ifname "$WIFI_IF" con-name "$HOTSPOT_NAME" \
    ssid "$HOTSPOT_NAME" mode ap \
    wifi.band bg wifi.channel 6 \
    802-11-wireless-security.key-mgmt wpa-psk \
    802-11-wireless-security.psk "$HOTSPOT_PASSWORD" \
    ipv4.method shared ipv6.method ignore autoconnect yes
fi

sudo nmcli connection up "$HOTSPOT_NAME" \
  || echo "⚠️ Hotspot could not start now (will start after reboot)."

echo "✅ Hotspot configured successfully."
