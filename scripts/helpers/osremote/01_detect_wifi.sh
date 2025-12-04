#!/usr/bin/env bash
set -e

if [[ -z "$WIFI_IF" ]]; then
    echo "❌ WIFI_IF environment variable is missing."
    exit 1
fi

echo "✅ Wi-Fi interface detected: $WIFI_IF"
