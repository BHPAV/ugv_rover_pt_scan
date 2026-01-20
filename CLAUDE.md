# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS 2 Humble autonomous rover system (Waveshare UGV Rover PT on Jetson Orin) that performs surveillance missions:
1. Sweeps gimbal through specified angles to scan surroundings
2. Captures frames at various gimbal poses
3. Saves JPEG frames with metadata
4. Offloads mission bundle to NAS via SSH + rsync over Tailscale
5. Returns to IDLE

## Hardware Platform

**Waveshare UGV Rover PT Jetson Orin Kit**

| Component | Specification |
|-----------|---------------|
| Host | Jetson Orin Nano (4GB/8GB) |
| Sub-controller | ESP32 running `ugv_base_ros` firmware |
| Gimbal | 2-DOF pan-tilt with SCServos |
| Camera | 160° 5MP USB camera on gimbal |
| LiDAR | D500 LiDAR (optional) |
| Depth | OAK-D Lite (optional) |
| Communication | GPIO UART `/dev/ttyTHS1` @ 115200 baud |
| Power | 3S LiPo with UPS module |

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
ros2 action send_goal /scan_and_offload ugv_rover_pt_interfaces/action/ScanAndOffload \
  "{duration_sec: 30.0, yaw_min_deg: -90.0, yaw_max_deg: 90.0, yaw_step_deg: 45.0, pitch_list_deg: [0.0, 30.0], frames_per_pose: 2, settle_ms: 500}"

# Install as systemd service (production deployment)
sudo ./scripts/install_systemd.sh
```

## Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                  Docker Container (ros:humble)                       │
│  ┌────────────────────────────────────────────────────────────────┐  │
│  │ v4l2_camera_node ──────────────────────→ /image_raw            │  │
│  │                                                                │  │
│  │ ugv_base_bridge (UART Owner)                                   │  │
│  │  ├─ /cmd_vel (Twist) ─────────────→ T=13 CMD_ROS_CTRL         │  │
│  │  ├─ /gimbal/setpoint (Vec3) ──────→ T=133 Gimbal command      │  │
│  │  ├─ T=1001 feedback ──────────────→ /ugv_base/feedback/json   │  │
│  │  └─ /ugv_base/ping service                                    │  │
│  │       ↓                                                        │  │
│  │  [Serial /dev/ttyTHS1] ←→ ESP32 (ugv_base_ros firmware)       │  │
│  │                                                                │  │
│  │ scan_mission_node (Action Server)                              │  │
│  │  ├─ Subscribes: /image_raw                                    │  │
│  │  ├─ Publishes: /gimbal/setpoint                               │  │
│  │  ├─ Action: /scan_and_offload                                 │  │
│  │  └─ Calls: offload_with_markers() → rsync                     │  │
│  └────────────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────────┘
```

## ROS 2 Packages

| Package | Type | Description |
|---------|------|-------------|
| `ugv_base` | Python | **UART owner** - ESP32 bridge for motors (T=13) and gimbal (T=133) |
| `ugv_rover_pt_bringup` | Python | Launch orchestration via `bringup.launch.py` |
| `ugv_rover_pt_scan_mission` | Python | Scan action server - coordinates capture and offload |
| `ugv_rover_pt_offload` | Python | Rsync helper with retry logic and marker files |
| `ugv_rover_pt_interfaces` | CMake | Action/message definitions |
| `ugv_rover_pt_gimbal_serial` | Python | (Legacy) Standalone gimbal driver - replaced by ugv_base |

## ESP32 JSON Protocol Reference

### TX Commands (Jetson → ESP32)

| Type | Name | Fields | Description |
|------|------|--------|-------------|
| `T=4` | Module Select | `cmd` (0=None, 1=RoArm, 2=Gimbal) | Select attached module |
| `T=13` | CMD_ROS_CTRL | `X` (m/s), `Z` (rad/s) | Velocity command |
| `T=40` | Gimbal Basic | `X` (yaw°), `Y` (pitch°) | Simple gimbal position |
| `T=41` | Gimbal Continuous | `X`, `Y`, `SX`, `SY` | Continuous movement with speed |
| `T=42` | Gimbal Stop | (none) | Stop gimbal movement |
| `T=131` | Feedback Flow | `cmd` (0=off, 1=on) | Enable/disable feedback stream |
| `T=133` | Gimbal Simple | `X`, `Y`, `SPD`, `ACC` | Position with speed/accel |
| `T=136` | Heartbeat | `cmd` (timeout_ms) | Set heartbeat watchdog |
| `T=142` | Feedback Interval | `cmd` (interval_ms) | Set feedback rate |
| `T=143` | UART Echo | `cmd` (0=off, 1=on) | Enable/disable echo mode |

### RX Feedback (ESP32 → Jetson)

| Type | Name | Fields | Description |
|------|------|--------|-------------|
| `T=1001` | Sensor Feedback | `L`, `R`, `ax/ay/az`, `gx/gy/gz`, `mx/my/mz`, `odl/odr`, `v`, `pan`, `tilt` | Full sensor data |
| `T=1005` | Command ACK | `id`, `status` (0=success) | Command acknowledgment |

### Protocol Notes

- **Format**: Newline-delimited JSON over UART
- **Baud rate**: 115200
- **Heartbeat**: ESP32 stops motors if no command received within timeout
- **Gimbal ranges**: Yaw ±180°, Pitch -30° to +90°
- **pan/tilt in feedback**: IMU-derived angles (for stabilization), not servo positions

## Key Configuration

### params.yaml Structure

```yaml
# Camera (v4l2_camera)
camera:
  ros__parameters:
    video_device: "/dev/video0"
    image_size: [1280, 720]

# ESP32 Bridge (ugv_base)
ugv_base_bridge:
  ros__parameters:
    serial:
      device: "/dev/ttyTHS1"
      baud: 115200
    heartbeat:
      timeout_ms: 1000
    ros_ctrl:
      topic: "/cmd_vel"
      send_rate_hz: 20.0
    gimbal:
      topic: "/gimbal/setpoint"
      default_spd: 200
      default_acc: 10

# Scan Mission
scan_mission:
  ros__parameters:
    mission_root: "/data/missions"
    nas_host: "192.168.1.20"
    nas_user: "boxhead"
    nas_dest_dir: "/share/boxhead/ugv_rover_pt/missions/"
```

### Environment Variables (.env)

| Variable | Purpose | Default |
|----------|---------|---------|
| `UGV_SERIAL_PORT` | ESP32 UART device | `/dev/ttyTHS1` |
| `UGV_CAMERA_DEVICE` | V4L2 camera | `/dev/video0` |
| `NAS_HOST` | Tailscale hostname | - |
| `NAS_USER` | SSH username | - |
| `NAS_DEST_DIR` | Destination path | - |

## Data Flow

1. **Mission Start**: Action client sends goal to `/scan_and_offload`
2. **Gimbal Sweep**: scan_mission publishes to `/gimbal/setpoint` → ugv_base sends T=133
3. **Frame Capture**: scan_mission subscribes to `/image_raw` → saves JPEG
4. **Package**: Creates `meta.json` with pose data
5. **Offload**: rsync to NAS with `.uploading`/`.complete` markers
6. **Complete**: Returns to IDLE state

## Development Standards

### Package Structure (ament_python)

```
ros2_ws/src/<package_name>/
├── package.xml              # Dependencies
├── setup.py                 # Entry points
├── setup.cfg                # CRITICAL: install_scripts path
├── resource/<package_name>  # Ament marker (empty file)
└── <package_name>/
    ├── __init__.py
    └── <node>.py
```

**Important**: Always include `setup.cfg` with:
```ini
[develop]
script_dir=$base/lib/<package_name>
[install]
install_scripts=$base/lib/<package_name>
```

### Code Style

- Python 3.10+ with type hints
- Dataclasses for configuration structures
- Thread-safe patterns for serial I/O
- Logging via `self.get_logger()`

### Testing

```bash
# Health check
ros2 service call /ugv_base/ping std_srvs/srv/Trigger

# Gimbal test
ros2 topic pub --once /gimbal/setpoint geometry_msgs/msg/Vector3 "{x: 45.0, y: 0.0}"

# Monitor feedback
ros2 topic echo /ugv_base/feedback/json

# Full scan test
ros2 action send_goal --feedback /scan_and_offload ugv_rover_pt_interfaces/action/ScanAndOffload \
  "{duration_sec: 10.0, yaw_min_deg: -45.0, yaw_max_deg: 45.0, yaw_step_deg: 45.0, pitch_list_deg: [0.0], frames_per_pose: 1, settle_ms: 500}"
```

## Jetson Serial Ports

| Device | Description | Notes |
|--------|-------------|-------|
| `/dev/ttyTHS0` | UART0 | Often debug console |
| `/dev/ttyTHS1` | UART1 | **ESP32 on UGV Rover PT** |
| `/dev/ttyTHS2` | UART2 | Available |
| `/dev/ttyUSB*` | USB serial | External adapters |

## Troubleshooting

### Serial Issues
- Verify user is in `dialout` group
- Check device exists: `ls -la /dev/ttyTHS*`
- Only one process can own the serial port

### Gimbal Not Moving
- Check ugv_base TX count via ping service
- Verify T=4 module selection sent at startup
- Confirm gimbal has power

### Build Errors
- Clean build: `rm -rf build/ install/ log/`
- Check `setup.cfg` exists for Python packages
- Verify dependencies in `package.xml`
