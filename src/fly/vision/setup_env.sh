#!/bin/bash


set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$BASH_SOURCE[0]}")" && pwd)"
VISION_REQUIREMENTS="${SCRIPT_DIR}/requirements-vision.txt"

if [ -z "${VIRTUAL_ENV:-}" ]; then # :-fallback is necessary for the if statement to trigger when VIRTUAL_ENV dne
    echo "No active virtual environment (\$VIRTUAL_ENV is unset)."
    echo "Activate your project venv first, then rerun this script."
    exit 1
fi

echo "Installing vision/detection dependencies into ${VIRTUAL_ENV}..."
uv pip install -r "${VISION_REQUIREMENTS}"

echo ""
echo "Setup complete. ${VIRTUAL_ENV} now has the full base + vision stack."
echo ""
