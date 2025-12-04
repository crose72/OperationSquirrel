#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

"$SCRIPT_DIR/setup_env.sh" "$@"
"$SCRIPT_DIR/setup_docker.sh"
"$SCRIPT_DIR/setup_nvidia_container_toolkit.sh"
"$SCRIPT_DIR/setup_systemd_services.sh"
"$SCRIPT_DIR/setup_hwio.sh"

echo "✔ Setup complete!
Please run:
  source ~/.bashrc
  sudo reboot
"
