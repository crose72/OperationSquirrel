#!/usr/bin/env bash
set -euo pipefail

echo "🔧 Starting MCAP → CSV conversion..."

# --- Parse arguments ---
PROTO_DIR="proto"  # default
ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --proto-dir|--proto)
            PROTO_DIR="$2"
            shift 2
            ;;
        *)
            ARGS+=("$1")
            shift
            ;;
    esac
done

# --- Check for active Python virtual environment ---
if [ -z "${VIRTUAL_ENV:-}" ]; then
    echo "⚠️  No Python virtual environment detected."
    echo ""
    echo "👉 To continue, please activate one first:"
    echo "   source ~/path/to/os-venv/bin/activate"
    echo ""
    echo "💡 Or create a new one if needed:"
    echo "   python3 -m venv os-venv"
    echo "   source os-venv/bin/activate"
    echo ""
    echo "Exiting to avoid installing system-wide packages."
    exit 1
fi

echo "✅ Using venv at: $VIRTUAL_ENV"

# --- Ensure dependencies are installed (quietly) ---
if ! python3 -c "import grpc_tools, pandas, mcap" &>/dev/null; then
    echo "📦 Installing Python dependencies..."
    pip install -q --upgrade pip
    pip install -q protobuf grpcio-tools pandas mcap
fi

# --- Regenerate protobufs if needed ---
if [ -d "$PROTO_DIR" ]; then
    echo "🧩 Checking for proto bindings in: $PROTO_DIR"
    NEED_GEN=false
    for proto in "$PROTO_DIR"/*.proto; do
        [ ! -f "${proto%.proto}_pb2.py" ] && NEED_GEN=true
    done

    if $NEED_GEN; then
        echo "🔄 Generating Python protobuf bindings..."
        python3 -m grpc_tools.protoc -I"$PROTO_DIR" --python_out="$PROTO_DIR" "$PROTO_DIR"/*.proto
    else
        echo "✅ Proto bindings are up to date."
    fi
else
    echo "❌ Proto directory not found: $PROTO_DIR"
    exit 1
fi

# --- Run your converter ---
echo "🚀 Running MCAP to CSV conversion..."
SCRIPT_DIR="$(dirname "$(realpath "$0")")"
export PROTO_DIR="$PROTO_DIR"
python3 "$SCRIPT_DIR/helpers/gen_test_csv_from_mcap.py" "${ARGS[@]}"


echo "✅ Conversion complete."
