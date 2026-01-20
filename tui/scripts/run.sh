#!/usr/bin/env bash
#
# Run the UGV Rover PT TUI
#
# Usage:
#   ./scripts/run.sh                  # Default: ws://ubuntu:9090
#   ./scripts/run.sh ws://host:9090   # Custom URL
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TUI_DIR="$(dirname "$SCRIPT_DIR")"

cd "$TUI_DIR"

# Check if bun is installed
if ! command -v bun &>/dev/null; then
    echo "Error: Bun is not installed."
    echo "Install it with: curl -fsSL https://bun.sh/install | bash"
    exit 1
fi

# Install dependencies if needed
if [ ! -d "node_modules" ]; then
    echo "Installing dependencies..."
    bun install
fi

# Run the TUI
if [ -n "$1" ]; then
    exec bun run src/index.tsx --url "$1"
else
    exec bun run src/index.tsx
fi
