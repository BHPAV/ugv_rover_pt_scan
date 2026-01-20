#!/usr/bin/env bash
#
# UGV Rover PT Monitor - Auto-opens TUI when UGV comes online
#
# This script monitors for the Jetson (via Tailscale) and opens
# a Terminal window with the status TUI when it becomes available.
#
# Install as launchd agent:
#   ./macos/install_monitor.sh
#

set -e

# Configuration
UGV_HOST="${UGV_HOST:-ubuntu}"  # Tailscale MagicDNS hostname
UGV_USER="${UGV_USER:-jetson}"
CHECK_INTERVAL=10               # Seconds between checks when offline
BOOT_WAIT=30                    # Seconds to wait after host detected for Docker
CONTAINER_NAME="ugv_ros"
LOG_FILE="$HOME/.ugv_monitor.log"

# State tracking
STATE_FILE="$HOME/.ugv_monitor_state"
LAST_STATE="unknown"

log() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1" >> "$LOG_FILE"
}

# Load last known state
if [[ -f "$STATE_FILE" ]]; then
    LAST_STATE=$(cat "$STATE_FILE")
fi

check_host_online() {
    # Quick ping check via Tailscale
    ping -c 1 -W 2 "$UGV_HOST" &>/dev/null
}

check_container_ready() {
    # Check if container is running and ROS nodes are up
    ssh -o ConnectTimeout=5 -o BatchMode=yes "${UGV_USER}@${UGV_HOST}" \
        "docker ps --format '{{.Names}}' | grep -q '^${CONTAINER_NAME}\$' && \
         docker exec ${CONTAINER_NAME} bash -c 'source /work/ros2_ws/install/setup.bash && ros2 node list 2>/dev/null | grep -q ugv_base_bridge'" \
        &>/dev/null
}

open_tui_terminal() {
    log "Opening TUI in new Terminal window"

    # AppleScript to open Terminal with SSH command
    osascript <<EOF
tell application "Terminal"
    activate
    set newTab to do script "echo 'Connecting to UGV Rover PT...' && ssh -t ${UGV_USER}@${UGV_HOST} 'docker exec -it ${CONTAINER_NAME} bash -c \"source /work/ros2_ws/install/setup.bash && python3 /work/scripts/status_tui.py\"'"
    set custom title of front window to "UGV Rover PT Status"
end tell
EOF
}

notify_user() {
    local title="$1"
    local message="$2"
    osascript -e "display notification \"$message\" with title \"$title\""
}

main_loop() {
    log "UGV Monitor started - watching for $UGV_HOST"

    while true; do
        if check_host_online; then
            if [[ "$LAST_STATE" != "online" ]]; then
                log "Host $UGV_HOST detected - waiting ${BOOT_WAIT}s for services"
                notify_user "UGV Rover PT" "Device detected, waiting for services..."

                # Wait for boot to complete
                sleep "$BOOT_WAIT"

                # Wait for container to be ready (with timeout)
                READY_TIMEOUT=120
                READY_WAIT=0
                while ! check_container_ready; do
                    if [[ $READY_WAIT -ge $READY_TIMEOUT ]]; then
                        log "Timeout waiting for container - will retry"
                        break
                    fi
                    log "Waiting for container to be ready..."
                    sleep 5
                    READY_WAIT=$((READY_WAIT + 5))
                done

                if check_container_ready; then
                    log "Container ready - opening TUI"
                    notify_user "UGV Rover PT" "Connected! Opening status monitor..."
                    open_tui_terminal
                    LAST_STATE="online"
                    echo "online" > "$STATE_FILE"
                fi
            fi
            sleep "$CHECK_INTERVAL"
        else
            if [[ "$LAST_STATE" == "online" ]]; then
                log "Host $UGV_HOST went offline"
                notify_user "UGV Rover PT" "Device disconnected"
                LAST_STATE="offline"
                echo "offline" > "$STATE_FILE"
            fi
            sleep "$CHECK_INTERVAL"
        fi
    done
}

# Handle signals gracefully
trap 'log "Monitor stopped"; exit 0' SIGTERM SIGINT

# Run main loop
main_loop
