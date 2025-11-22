#!/usr/bin/env bash
# ==============================================================
# 🐿️ Operation Squirrel — GHCR Push Helper
# --------------------------------------------------------------
# Pushes your local OS-Dev Docker image to GitHub Container Registry.
#
# Usage:
#   ./scripts/push_to_ghcr.sh
#
# Requirements:
#   export GHCR_PAT="your-token-here"
# ==============================================================

set -e

IMAGE_LOCAL="crose72/os-dev:cuda11.8-trt853-cv4.10-sm61-sm75-sm86-mcap-ubuntu22"
IMAGE_REMOTE="ghcr.io/crose72/os-dev:cuda11.8-trt853-cv4.10-sm61-sm75-sm86-mcap-ubuntu22"

echo "🐿️ GHCR Push Script — Operation Squirrel"
echo "-----------------------------------------"

# ----------------------------------------------------------------------
# 1. Ensure PAT exists
# ----------------------------------------------------------------------
if [ -z "$GHCR_PAT" ]; then
    echo "❌ ERROR: Environment variable GHCR_PAT is not set."
    echo "   Run this first:"
    echo "     export GHCR_PAT=ghp_yourtokenhere"
    exit 1
fi

# ----------------------------------------------------------------------
# 2. Login to GHCR (only if not already logged in)
# ----------------------------------------------------------------------
echo "🔐 Logging into GHCR..."
echo "$GHCR_PAT" | docker login ghcr.io -u crose72 --password-stdin

echo "✅ Login OK"

# ----------------------------------------------------------------------
# 3. Check local image exists
# ----------------------------------------------------------------------
if [[ "$(docker images -q $IMAGE_LOCAL 2>/dev/null)" == "" ]]; then
    echo "❌ ERROR: Local image not found:"
    echo "   $IMAGE_LOCAL"
    exit 1
fi

echo "🐋 Found local image: $IMAGE_LOCAL"

# ----------------------------------------------------------------------
# 4. Tag the image for GHCR
# ----------------------------------------------------------------------
echo "🏷️ Tagging image..."
docker tag "$IMAGE_LOCAL" "$IMAGE_REMOTE"

echo "✅ Tagged:"
echo "   $IMAGE_REMOTE"

# ----------------------------------------------------------------------
# 5. Push to GHCR
# ----------------------------------------------------------------------
echo "🚀 Pushing image to GHCR..."
docker push "$IMAGE_REMOTE"

echo ""
echo "🎉 SUCCESS: Image pushed to GHCR!"
echo "📦 $IMAGE_REMOTE"
echo ""
echo "You can now use this image in GitHub Actions for ultra-fast pulls."
