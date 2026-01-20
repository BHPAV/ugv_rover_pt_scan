#!/usr/bin/env bash
#
# Run the UGV Rover PT Status TUI
#
# Usage:
#   ./scripts/run_status_tui.sh
#
# This launches the TUI inside the running Docker container.
# The container must already be running (via run_bringup.sh).
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Check if container is running
if ! docker ps --format '{{.Names}}' | grep -q '^ugv_ros$'; then
    echo "Error: Container 'ugv_ros' is not running."
    echo "Start it first with: ./scripts/run_bringup.sh"
    exit 1
fi

# Run the TUI
exec docker exec -it ugv_ros bash -c "source /work/ros2_ws/install/setup.bash && python3 /work/scripts/status_tui.py"
