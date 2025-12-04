#!/usr/bin/env bash
set -e

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

sudo nmcli connection up "$HOTSPOT_NAME" \
  || echo "⚠️ Could not start hotspot now; may require reboot."

echo "✅ Hotspot configured: $HOTSPOT_NAME"
