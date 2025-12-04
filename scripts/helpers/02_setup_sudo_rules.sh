#!/usr/bin/env bash
set -e

{
  echo "$USER_NAME ALL=(ALL) NOPASSWD: $SCRIPT_PATH"
  echo "$USER_NAME ALL=(ALL) NOPASSWD: /bin/rm -rf $DATA_PATH"
} | sudo tee "$SUDOERS_FILE" >/dev/null

sudo chmod 440 "$SUDOERS_FILE"

echo "✅ Passwordless sudo rules configured."
