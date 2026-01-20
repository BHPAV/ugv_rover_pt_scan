#!/usr/bin/env bash
#
# Install UGV Monitor as macOS launchd agent
#
# This sets up automatic TUI launch when the UGV comes online.
#
# Usage:
#   ./macos/install_monitor.sh          # Install and start
#   ./macos/install_monitor.sh remove   # Uninstall
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
PLIST_NAME="com.ugv.monitor"
PLIST_SRC="$SCRIPT_DIR/$PLIST_NAME.plist"
PLIST_DEST="$HOME/Library/LaunchAgents/$PLIST_NAME.plist"

# Install to Application Support to avoid Downloads folder restrictions
INSTALL_DIR="$HOME/Library/Application Support/UGV-Monitor"
MONITOR_SCRIPT="$INSTALL_DIR/ugv_monitor.sh"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

info() { echo -e "${GREEN}[INFO]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

install_monitor() {
    info "Installing UGV Monitor..."

    # Check prerequisites
    if ! command -v ssh &>/dev/null; then
        error "SSH not found"
    fi

    # Create install directory
    mkdir -p "$INSTALL_DIR"

    # Copy monitor script to install location (avoids Downloads folder restrictions)
    info "Copying monitor script to $INSTALL_DIR..."
    cp "$SCRIPT_DIR/ugv_monitor.sh" "$MONITOR_SCRIPT"
    chmod +x "$MONITOR_SCRIPT"

    # Clear any quarantine attributes
    xattr -cr "$INSTALL_DIR" 2>/dev/null || true

    # Create LaunchAgents directory if needed
    mkdir -p "$HOME/Library/LaunchAgents"

    # Stop existing service if running
    if launchctl list | grep -q "$PLIST_NAME"; then
        info "Stopping existing service..."
        launchctl unload "$PLIST_DEST" 2>/dev/null || true
    fi

    # Generate plist with correct paths
    info "Generating launchd plist..."
    sed -e "s|__INSTALL_PATH__|$INSTALL_DIR/..|g" \
        -e "s|__HOME__|$HOME|g" \
        "$PLIST_SRC" > "$PLIST_DEST"

    # Fix the path in plist to point to installed script
    sed -i '' "s|$INSTALL_DIR/../macos/ugv_monitor.sh|$MONITOR_SCRIPT|g" "$PLIST_DEST"

    # Load the service
    info "Loading launchd agent..."
    launchctl load "$PLIST_DEST"

    # Verify it's running
    sleep 1
    if launchctl list | grep -q "$PLIST_NAME"; then
        info "UGV Monitor installed and running!"
        echo ""
        echo "The monitor will:"
        echo "  - Run automatically at login"
        echo "  - Watch for the UGV to come online (via Tailscale)"
        echo "  - Open a Terminal with the status TUI when detected"
        echo ""
        echo "Logs: ~/.ugv_monitor.log"
        echo ""
        echo "To uninstall: $0 remove"
    else
        error "Failed to start service. Check ~/.ugv_monitor_stderr.log"
    fi
}

remove_monitor() {
    info "Removing UGV Monitor..."

    if launchctl list | grep -q "$PLIST_NAME"; then
        info "Stopping service..."
        launchctl unload "$PLIST_DEST" 2>/dev/null || true
    fi

    if [[ -f "$PLIST_DEST" ]]; then
        rm "$PLIST_DEST"
        info "Removed launchd plist"
    fi

    # Remove installed script
    if [[ -d "$INSTALL_DIR" ]]; then
        rm -rf "$INSTALL_DIR"
        info "Removed install directory"
    fi

    # Clean up state files
    rm -f "$HOME/.ugv_monitor_state"
    rm -f "$HOME/.ugv_monitor.log"
    rm -f "$HOME/.ugv_monitor_stdout.log"
    rm -f "$HOME/.ugv_monitor_stderr.log"

    info "UGV Monitor uninstalled"
}

status_monitor() {
    echo "UGV Monitor Status"
    echo "=================="

    if launchctl list | grep -q "$PLIST_NAME"; then
        echo "Service: RUNNING"
        PID=$(launchctl list | grep "$PLIST_NAME" | awk '{print $1}')
        echo "PID: $PID"
    else
        echo "Service: STOPPED"
    fi

    if [[ -f "$HOME/.ugv_monitor_state" ]]; then
        echo "Last UGV state: $(cat "$HOME/.ugv_monitor_state")"
    fi

    echo ""
    echo "Recent log entries:"
    tail -5 "$HOME/.ugv_monitor.log" 2>/dev/null || echo "(no logs yet)"
}

# Main
case "${1:-install}" in
    install)
        install_monitor
        ;;
    remove|uninstall)
        remove_monitor
        ;;
    status)
        status_monitor
        ;;
    *)
        echo "Usage: $0 [install|remove|status]"
        exit 1
        ;;
esac
