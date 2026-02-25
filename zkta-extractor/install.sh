#!/bin/bash
# Install the ZKTA Extractor SAI into the Spring AI directory.
#
# Usage: ./install.sh [engine_dir]
#
# The AI will be installed to:
#   ~/.spring/AI/Skirmish/ZKTAExtractor/0.1/

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
AI_DIR="$HOME/.spring/AI/Skirmish/ZKTAExtractor/0.1"

# Build release if not already built
if [ ! -f "$SCRIPT_DIR/target/release/libSkirmishAI.so" ]; then
    echo "Building zkta-extractor (release)..."
    cd "$SCRIPT_DIR"
    cargo build --release
fi

echo "Installing to $AI_DIR"
mkdir -p "$AI_DIR"
cp "$SCRIPT_DIR/target/release/libSkirmishAI.so" "$AI_DIR/"
cp "$SCRIPT_DIR/data/AIInfo.lua" "$AI_DIR/"

echo "Installed:"
ls -la "$AI_DIR/"
echo "Done."
