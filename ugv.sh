#!/usr/bin/env bash
# ----------------------------------------
# UGV Rover PT - Unified CLI
# ----------------------------------------
# Single entry point for all common operations.
# Usage: ./ugv.sh <command> [args]

set -euo pipefail
cd "$(dirname "$0")"

COMPOSE_DIR="docker"
CONTAINER="ugv_ros"

usage() {
    cat <<EOF
UGV Rover PT Control Script

Usage: ./ugv.sh <command> [args]

Commands:
  build         Build the Docker container
  up            Start the system (build + run)
  down          Stop the system
  restart       Restart the container
  shell         Open a bash shell in the container
  logs          Follow container logs
  status        Show container and ROS node status
  mission       Run a scan mission (optional: duration in seconds)
  probe         Run system probe on Jetson host

Examples:
  ./ugv.sh up                    # Start the system
  ./ugv.sh shell                 # Enter container shell
  ./ugv.sh logs                  # Follow logs
  ./ugv.sh mission 30            # Run 30-second mission
  ./ugv.sh mission               # Run default mission
EOF
}

cmd_build() {
    echo "Building container..."
    cd "$COMPOSE_DIR"
    docker compose build
}

cmd_up() {
    echo "Starting UGV system..."
    cd "$COMPOSE_DIR"
    docker compose up -d --build
    echo "Waiting for nodes to initialize..."
    sleep 5
    cmd_status
}

cmd_down() {
    echo "Stopping UGV system..."
    cd "$COMPOSE_DIR"
    docker compose down
}

cmd_restart() {
    echo "Restarting container..."
    cd "$COMPOSE_DIR"
    docker compose restart "$CONTAINER"
}

cmd_shell() {
    docker exec -it "$CONTAINER" bash
}

cmd_logs() {
    docker logs -f "$CONTAINER"
}

cmd_status() {
    echo "=== Container Status ==="
    docker ps --filter "name=$CONTAINER" --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"
    echo ""
    echo "=== ROS 2 Nodes ==="
    docker exec "$CONTAINER" bash -c 'source /opt/ros/humble/setup.bash && source /work/ros2_ws/install/setup.bash 2>/dev/null && ros2 node list' 2>/dev/null || echo "(nodes not ready yet)"
    echo ""
    echo "=== ROS 2 Topics ==="
    docker exec "$CONTAINER" bash -c 'source /opt/ros/humble/setup.bash && source /work/ros2_ws/install/setup.bash 2>/dev/null && ros2 topic list' 2>/dev/null || echo "(topics not ready yet)"
}

cmd_mission() {
    local duration="${1:-30}"
    echo "Running scan mission (duration: ${duration}s)..."
    docker exec "$CONTAINER" bash -c "
        source /opt/ros/humble/setup.bash &&
        source /work/ros2_ws/install/setup.bash &&
        ros2 action send_goal --feedback /scan_and_offload \
            ugv_rover_pt_interfaces/action/ScanAndOffload \
            \"{duration_sec: ${duration}.0}\"
    "
}

cmd_probe() {
    if [[ -f "ugv_probe.py" ]]; then
        echo "Running system probe..."
        python3 ugv_probe.py --target "$(hostname)" --out ugv_probe_local.json
    else
        echo "Error: ugv_probe.py not found"
        exit 1
    fi
}

# Main dispatch
case "${1:-}" in
    build)   cmd_build ;;
    up)      cmd_up ;;
    down)    cmd_down ;;
    restart) cmd_restart ;;
    shell)   cmd_shell ;;
    logs)    cmd_logs ;;
    status)  cmd_status ;;
    mission) cmd_mission "${2:-30}" ;;
    probe)   cmd_probe ;;
    -h|--help|help|"")
        usage
        exit 0
        ;;
    *)
        echo "Unknown command: $1"
        usage
        exit 1
        ;;
esac
