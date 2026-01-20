# UGV Rover PT Scan - Provisioning Guide

This document covers the complete provisioning process for the Waveshare UGV Rover PT with Jetson Orin, running ROS 2 Humble for autonomous scan missions.

## Table of Contents

1. [System Overview](#system-overview)
2. [Hardware Requirements](#hardware-requirements)
3. [Network Configuration](#network-configuration)
4. [Provisioning Phases](#provisioning-phases)
5. [Waveshare ESP32 Protocol](#waveshare-esp32-protocol)
6. [Troubleshooting](#troubleshooting)

---

## System Overview

The UGV Rover PT Scan system performs autonomous surveillance missions:
1. Sweeps gimbal through specified angles
2. Captures frames at each pose
3. Saves JPEG frames with metadata to local storage
4. Offloads mission bundle to NAS via rsync over Tailscale
5. Returns to IDLE state

### Architecture

```
┌─────────────────────────────────────────────────────────────┐
│              Jetson Orin (Docker Container)                 │
│  ┌────────────────────────────────────────────────────────┐ │
│  │  v4l2_camera_node ──────→ /image_raw                   │ │
│  │                                                        │ │
│  │  gimbal_node ←────────── /gimbal/setpoint              │ │
│  │       │                                                │ │
│  │       ↓ JSON over UART                                 │ │
│  │  [/dev/ttyTHS1] ──→ ESP32 ──→ Gimbal Servos           │ │
│  │                                                        │ │
│  │  scan_mission_node (Action Server)                     │ │
│  │       ├─ Subscribes: /image_raw                        │ │
│  │       ├─ Publishes: /gimbal/setpoint                   │ │
│  │       ├─ Action: /scan_and_offload                     │ │
│  │       └─ Offload: rsync → NAS                          │ │
│  └────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

---

## Hardware Requirements

| Component | Specification |
|-----------|---------------|
| **Compute** | Jetson Orin Nano/NX |
| **Platform** | Waveshare UGV Rover PT |
| **Camera** | USB UVC camera (tested: generic USB Camera) |
| **Gimbal** | Waveshare PT gimbal (2-axis: pan/tilt) |
| **Controller** | ESP32 (Waveshare firmware) |
| **Serial** | /dev/ttyTHS1 @ 115200 baud |

### Device Paths

| Device | Default Path | Environment Variable |
|--------|--------------|---------------------|
| Camera | `/dev/video0` | `UGV_CAMERA_DEVICE` |
| ESP32 Serial | `/dev/ttyTHS1` | `UGV_SERIAL_PORT` |

---

## Network Configuration

### Tailscale VPN

The system uses Tailscale for secure connectivity to the NAS:

```bash
# Install Tailscale
curl -fsSL https://tailscale.com/install.sh | sh

# Authenticate
sudo tailscale up

# Verify connectivity
tailscale status
ping box-nas  # NAS hostname
```

### SSH Key Setup for NAS Offload

```bash
# Generate SSH key (if needed)
ssh-keygen -t ed25519 -f ~/.ssh/id_ed25519 -N ""

# Copy to NAS
ssh-copy-id -i ~/.ssh/id_ed25519 boxhead@box-nas

# Test connection
ssh boxhead@box-nas "echo 'SSH OK'"
```

---

## Provisioning Phases

### Phase 0: Baseline Capture

```bash
# Create log directory
LOG_DIR="$HOME/ugv_setup_logs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"

# Capture system info
uname -a > "$LOG_DIR/baseline.txt"
df -h >> "$LOG_DIR/baseline.txt"
free -h >> "$LOG_DIR/baseline.txt"
ls -l /dev/ttyTHS* /dev/video* >> "$LOG_DIR/baseline.txt" 2>&1
```

### Phase 1: Remove Waveshare Conflicts

The Waveshare default installation runs `app.py` on boot, which occupies the camera and serial devices.

```bash
# Check for conflicting processes
ps aux | grep -E "app.py|waveshare" | grep -v grep

# Check device usage
fuser -v /dev/video0
fuser -v /dev/ttyTHS1

# Kill conflicting processes
pkill -f app.py

# Remove from crontab
crontab -l > /tmp/crontab.bak
crontab -l | grep -v "app.py" | crontab -
```

### Phase 2: Install Prerequisites

```bash
# Verify Docker
docker --version
docker compose version

# Install required packages
apt-get update
apt-get install -y rsync openssh-client python3-pip

# Install Tailscale (if needed)
curl -fsSL https://tailscale.com/install.sh | sh
```

### Phase 3: Deploy Codebase

```bash
# Create project directory
sudo mkdir -p /opt/ugv_rover_pt_scan
sudo chown $USER:$USER /opt/ugv_rover_pt_scan

# Copy from local machine (run on Mac/PC)
scp -r /path/to/ugv_rover_pt_scan/* jetson@<IP>:/opt/ugv_rover_pt_scan/

# Create .env from template
cp /opt/ugv_rover_pt_scan/.env.example /opt/ugv_rover_pt_scan/.env
# Edit .env with correct values
```

### Phase 4: Configure Environment

Create `/opt/ugv_rover_pt_scan/.env`:

```bash
# ROS
ROS_DOMAIN_ID=0

# Serial (ESP32/Gimbal)
UGV_SERIAL_PORT=/dev/ttyTHS1
UGV_SERIAL_BAUD=115200

# Camera
UGV_CAMERA_DEVICE=/dev/video0
UGV_CAMERA_FPS=30
UGV_CAMERA_WIDTH=1280
UGV_CAMERA_HEIGHT=720

# Mission storage
UGV_MISSION_ROOT=/data/missions

# NAS Offload
NAS_HOST=box-nas
NAS_USER=boxhead
NAS_DEST_DIR=/share/boxhead/ugv_rover_pt/missions/
```

### Phase 5: Build and Start

```bash
cd /opt/ugv_rover_pt_scan/docker

# Build and start container
docker compose up -d --build

# Verify nodes are running
docker exec ugv_ros bash -lc "source /work/ros2_ws/install/setup.bash && ros2 node list"
```

Expected nodes:
- `/v4l2_camera_node`
- `/gimbal_node`
- `/scan_mission_node`

### Phase 6: Install Systemd Service

```bash
# Copy service file
sudo cp /opt/ugv_rover_pt_scan/systemd/ugv-rover-pt.service /etc/systemd/system/

# Enable and start
sudo systemctl daemon-reload
sudo systemctl enable ugv-rover-pt.service
sudo systemctl start ugv-rover-pt.service

# Verify
systemctl status ugv-rover-pt.service
```

### Phase 7: Verify Hardware

```bash
# Enter container
docker exec -it ugv_ros bash

# Source ROS
source /work/ros2_ws/install/setup.bash

# Test camera
ros2 topic echo /image_raw --once

# Test gimbal
ros2 topic pub --once /gimbal/setpoint geometry_msgs/msg/Vector3 "{x: 30.0, y: 0.0, z: 0.0}"
ros2 topic pub --once /gimbal/setpoint geometry_msgs/msg/Vector3 "{x: -30.0, y: 0.0, z: 0.0}"
ros2 topic pub --once /gimbal/setpoint geometry_msgs/msg/Vector3 "{x: 0.0, y: 0.0, z: 0.0}"
```

### Phase 8: Test Mission

```bash
ros2 action send_goal /scan_and_offload ugv_rover_pt_interfaces/action/ScanAndOffload \
  "{duration_sec: 30.0, yaw_min_deg: -45.0, yaw_max_deg: 45.0, yaw_step_deg: 45.0, pitch_list_deg: [0.0], frames_per_pose: 2, settle_ms: 500}" \
  --feedback
```

---

## Waveshare ESP32 Protocol

The Waveshare UGV uses JSON commands over UART to control the gimbal and base motors.

### Serial Configuration

| Parameter | Value |
|-----------|-------|
| Port | `/dev/ttyTHS1` |
| Baud Rate | 115200 |
| Format | JSON + newline (`\n`) |

### Initialization Sequence (REQUIRED)

**Critical**: The ESP32 must be initialized before gimbal commands are accepted.

```python
# Must send these commands on startup, in order:
{"T":142,"cmd":50}   # Set feedback interval (50ms)
{"T":131,"cmd":1}    # Enable serial feedback
{"T":143,"cmd":0}    # Disable serial echo
{"T":4,"cmd":2}      # SELECT MODULE: 2 = Gimbal mode
```

Module types for `T:4`:
- `0` = None
- `1` = RoArm-M2-S
- `2` = Gimbal (required for PT control)

### Command Reference

| Command | T Value | Parameters | Description |
|---------|---------|------------|-------------|
| Emergency Stop | 0 | none | `{"T":0}` |
| Base Motor | 1 | L, R | `{"T":1,"L":speed,"R":speed}` |
| Lights | 132 | IO4, IO5 | `{"T":132,"IO4":pwm,"IO5":pwm}` |
| Gimbal Control | 133 | X, Y, SPD, ACC | `{"T":133,"X":yaw,"Y":pitch,"SPD":speed,"ACC":accel}` |
| Gimbal Base | 141 | X, Y, SPD | `{"T":141,"X":yaw,"Y":pitch,"SPD":speed}` |
| Feedback Interval | 142 | cmd | `{"T":142,"cmd":ms}` |
| Echo Control | 143 | cmd | `{"T":143,"cmd":0/1}` |

### Gimbal Control (T=133)

```json
{"T":133,"X":yaw,"Y":pitch,"SPD":speed,"ACC":accel}
```

| Parameter | Range | Description |
|-----------|-------|-------------|
| X (yaw) | -180 to +180 | Horizontal rotation (degrees) |
| Y (pitch) | -30 to +90 | Vertical tilt (degrees) |
| SPD | 0-255 | Movement speed (0 = use default) |
| ACC | 0-255 | Acceleration (0 = use default) |

**Recommended values**: SPD=200, ACC=10

### Base Motor Control (T=1)

```json
{"T":1,"L":left_speed,"R":right_speed}
```

| Parameter | Range | Description |
|-----------|-------|-------------|
| L | -1.0 to +1.0 | Left motor speed (float, negative = reverse) |
| R | -1.0 to +1.0 | Right motor speed (float, negative = reverse) |

**Important**: Speed values are **floats** from -1.0 to 1.0, NOT integers.

Examples:
- Forward: `{"T":1,"L":0.5,"R":0.5}`
- Backward: `{"T":1,"L":-0.5,"R":-0.5}`
- Turn left: `{"T":1,"L":-0.3,"R":0.3}`
- Stop: `{"T":1,"L":0,"R":0}` or `{"T":0}`

### Python Example

```python
import serial
import json
import time

ser = serial.Serial('/dev/ttyTHS1', 115200, timeout=0.2)

def send_cmd(cmd):
    msg = (json.dumps(cmd) + '\n').encode('utf-8')
    ser.write(msg)
    ser.flush()

# Initialize gimbal mode
send_cmd({"T":142,"cmd":50})
send_cmd({"T":131,"cmd":1})
send_cmd({"T":143,"cmd":0})
send_cmd({"T":4,"cmd":2})
time.sleep(0.5)

# Move gimbal
send_cmd({"T":133,"X":45,"Y":0,"SPD":200,"ACC":10})
time.sleep(2)

# Move base forward briefly (floats -1.0 to 1.0)
send_cmd({"T":1,"L":0.4,"R":0.4})
time.sleep(0.5)
send_cmd({"T":0})  # Stop
```

---

## Troubleshooting

### Gimbal Not Responding

**Symptom**: Commands sent but gimbal doesn't move.

**Causes & Fixes**:

1. **Missing initialization**: Must send `{"T":4,"cmd":2}` to enable gimbal mode
2. **Missing newline**: Commands must end with `\n`
3. **SPD/ACC = 0**: Use SPD=200, ACC=10 for visible movement
4. **Wrong serial port**: Verify with `ls -l /dev/ttyTHS*`
5. **Device busy**: Check `fuser -v /dev/ttyTHS1`

### Camera Not Publishing

**Symptom**: `/image_raw` topic has no messages.

**Fixes**:
1. Check device exists: `ls -l /dev/video0`
2. Check device busy: `fuser -v /dev/video0`
3. Kill Waveshare app.py: `pkill -f app.py`
4. Verify in container: `v4l2-ctl --list-devices`

### Docker Build Fails

**Symptom**: Proxy timeout or network errors during build.

**Fixes**:
1. Remove stale proxy config:
   ```bash
   rm ~/.docker/config.json
   sudo rm /etc/systemd/system/docker.service.d/proxy.conf
   sudo systemctl daemon-reload
   sudo systemctl restart docker
   ```

### NumPy Compatibility Error

**Symptom**: `_ARRAY_API not found` error with cv_bridge.

**Fix**: Pin numpy<2 in Dockerfile:
```dockerfile
RUN pip3 install --no-cache-dir "numpy<2"
```

### Systemd Service Fails at Boot

**Symptom**: Service fails with DNS or network errors.

**Fix**: Add startup delay in service file:
```ini
[Service]
ExecStartPre=/bin/sleep 10
```

### SSH to NAS Fails

**Symptom**: rsync offload fails with permission denied.

**Fixes**:
1. Generate SSH key: `ssh-keygen -t ed25519`
2. Copy to NAS: `ssh-copy-id user@nas-host`
3. Test: `ssh -o BatchMode=yes user@nas-host echo OK`

---

## Quick Reference

### Verify System Health

```bash
# Check service
systemctl status ugv-rover-pt.service

# Check container
docker ps | grep ugv_ros

# Check ROS nodes
docker exec ugv_ros bash -lc "source /work/ros2_ws/install/setup.bash && ros2 node list"

# Check action server
docker exec ugv_ros bash -lc "source /work/ros2_ws/install/setup.bash && ros2 action list"
```

### Manual Gimbal Control

```bash
# Center
ros2 topic pub --once /gimbal/setpoint geometry_msgs/msg/Vector3 "{x: 0, y: 0, z: 0}"

# Look right 45°
ros2 topic pub --once /gimbal/setpoint geometry_msgs/msg/Vector3 "{x: 45, y: 0, z: 0}"

# Look up 30°
ros2 topic pub --once /gimbal/setpoint geometry_msgs/msg/Vector3 "{x: 0, y: 30, z: 0}"
```

### Trigger Mission

```bash
ros2 action send_goal /scan_and_offload ugv_rover_pt_interfaces/action/ScanAndOffload \
  "{duration_sec: 60.0, yaw_min_deg: -90.0, yaw_max_deg: 90.0, yaw_step_deg: 30.0, pitch_list_deg: [0.0, 30.0], frames_per_pose: 3, settle_ms: 500}" \
  --feedback
```
