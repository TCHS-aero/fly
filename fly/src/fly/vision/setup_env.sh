#!/bin/bash
set -e

ENV_NAME="ai"

# Initialize conda
eval "$(conda shell.bash hook)"

# Activate the ai environment
conda activate ${ENV_NAME}

# Install dependencies
echo "Installing dependencies..."
pip install --upgrade pip
pip install torch torchvision numpy supervision>=0.18.0 Pillow rfdetr

echo ""
echo "Setup complete!"
echo ""
echo "Environment 'ai' is now active with rfdetr dependencies installed."
echo ""
echo "To activate in the future: conda activate ai"
echo "To deactivate: conda deactivate"
