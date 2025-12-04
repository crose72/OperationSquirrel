#!/usr/bin/env bash
set -e

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
