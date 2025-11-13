#!/bin/bash
# setup_env.sh

# Read JSON and set environment variables for Jetson
set_jetson_env() {
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

    # Extract and set environment variables for jetson
    while IFS="=" read -r key value; do
        export "$key"="$value"
        echo "Set $key = $value"
    done < <(jq -r '.jetson | to_entries | .[] | .key + "=" + .value' "$json_file")

    echo "Jetson environment variables have been set successfully"
}

set_jetson_env
