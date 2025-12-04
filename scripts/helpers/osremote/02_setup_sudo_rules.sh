#!/usr/bin/env bash
set -e

# Verify required vars
if [[ -z "$USER_NAME" || -z "$SCRIPT_PATH" || -z "$DATA_PATH" || -z "$SUDOERS_FILE" ]]; then
    echo "❌ Missing required environment variables for sudo rules."
    exit 1
fi

echo "🔐 Configuring passwordless sudo for OSRemote..."

{
  echo "$USER_NAME ALL=(ALL) NOPASSWD: $SCRIPT_PATH"
  echo "$USER_NAME ALL=(ALL) NOPASSWD: /bin/rm -rf $DATA_PATH"
} | sudo tee "$SUDOERS_FILE" >/dev/null

sudo chmod 440 "$SUDOERS_FILE"

echo "✅ Passwordless sudo rules configured."
