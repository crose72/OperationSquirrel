#!/bin/bash
# setup_env_wsl.sh

# Read JSON and set environment variables for WSL
set_wsl_env() {
    # Check if jq is installed
    if ! command -v jq &> /dev/null; then
        echo "Error: jq is required but not installed. Please install it using:"
        echo "sudo apt-get install jq"
        exit 1
    }

    # Read the JSON file
    json_file="env_config.json"
    if [ ! -f "$json_file" ]; then
        echo "Error: $json_file not found"
        exit 1
    }

    # Extract and set environment variables for WSL
    while IFS="=" read -r key value; do
        export "$key"="$value"
        echo "Set $key = $value"
    done < <(jq -r '.wsl | to_entries | .[] | .key + "=" + .value' "$json_file")

    echo "WSL environment variables have been set successfully"
}

set_wsl_env
