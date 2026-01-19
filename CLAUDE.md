# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS 2 Humble autonomous rover system (Waveshare UGV Rover PT on Jetson Orin) that performs surveillance missions:
1. Sweeps gimbal through specified angles to scan surroundings
2. Captures frames at various gimbal poses
3. Saves JPEG frames with metadata
4. Offloads mission bundle to NAS via SSH + rsync over Tailscale
5. Returns to IDLE

## Build & Development Commands

```bash
# Build and run the full system (Docker)
./scripts/build_container.sh      # docker compose build
./scripts/run_bringup.sh          # docker compose up -d --build

# Enter container for interactive development
./scripts/enter_container.sh      # docker compose exec ugv_ros bash

# Inside container: rebuild ROS workspace
cd /work/ros2_ws && colcon build --symlink-install
source install/setup.bash

# Trigger a single scan mission (from host or container)
./scripts/run_mission_once.sh
# Or manually:
ros2 action send_goal /scan_and_offload ugv_rover_pt_scan_mission/action/ScanAndOffload \
  "{duration: 30, yaw_min: -90, yaw_max: 90, yaw_step: 45, pitch_angles: [0, 30], frames_per_pose: 2, settle_time: 0.5}"

# Install as systemd service (production deployment)
sudo ./scripts/install_systemd.sh
```

## Architecture

```
┌─────────────────────────────────────────────────────┐
│          Docker Container (ros:humble)              │
│  ┌───────────────────────────────────────────────┐  │
│  │ v4l2_camera_node ──→ /image_raw               │  │
│  │                                               │  │
│  │ gimbal_node ←─ /gimbal/setpoint               │  │
│  │  ↓                                            │  │
│  │ [Serial/UART] → ESP32/Waveshare Gimbal        │  │
│  │                                               │  │
│  │ scan_mission_node (Action Server)             │  │
│  │  ├─ Subscribes: /image_raw                    │  │
│  │  ├─ Publishes: /gimbal/setpoint               │  │
│  │  ├─ Action: /scan_and_offload                 │  │
│  │  └─ Calls: offload_with_markers() → rsync     │  │
│  └───────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────┘
```

**ROS 2 Packages** (in `ros2_ws/src/`):

- **ugv_rover_pt_bringup** (CMake): Launch orchestration - starts camera, gimbal, and mission nodes via `bringup.launch.py`
- **ugv_rover_pt_gimbal_serial** (Python): Waveshare PT gimbal serial driver - sends JSON commands over UART (T=133 gimbal type), subscribes to `/gimbal/setpoint` (Vector3: x=yaw, y=pitch)
- **ugv_rover_pt_scan_mission** (Python): Scan action server - coordinates camera capture, gimbal positioning, frame saving, and offload via `/scan_and_offload` action
- **ugv_rover_pt_offload** (Python utility): Rsync helper with retry logic and `.uploading`/`.complete` marker files

**Data Flow**:
- Mission frames saved to `/data/missions/mission_<timestamp>/frames/`
- Metadata in `meta.json`
- Offloaded to NAS via rsync+SSH with retry logic

## Key Configuration

Copy `.env.example` to `.env` and configure:

| Variable | Purpose |
|----------|---------|
| `UGV_SERIAL_PORT` | ESP32/gimbal UART (default: /dev/ttyTHS1) |
| `UGV_CAMERA_DEVICE` | V4L2 camera device |
| `NAS_HOST` | Tailscale hostname for offload target |
| `NAS_USER` | SSH username for rsync |
| `NAS_DEST_DIR` | Destination path on NAS |

## Gimbal Protocol

JSON over UART to ESP32/Waveshare:
```json
{"T":133, "X":yaw, "Y":pitch, "SPD":speed, "ACC":accel}
```
Clamped ranges: yaw ±180°, pitch -30° to 90°
