# UGV Rover PT Jetson Orin — Scan + Offload Mission (ROS 2 Humble)

Autonomous surveillance rover system for Waveshare UGV Rover PT on Jetson Orin. Boots into **IDLE** (camera + gimbal ready). On command, it:

1. Sweeps gimbal through specified angles to scan surroundings
2. Captures frames at each gimbal pose
3. Saves JPEG frames to `/data/missions/mission_<timestamp>/`
4. Offloads mission bundle to NAS over Tailscale via `rsync+ssh`
5. Returns to IDLE

## Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                  Docker Container (ros:humble)                       │
│  ┌────────────────────────────────────────────────────────────────┐  │
│  │ v4l2_camera_node ──────────────────────→ /image_raw            │  │
│  │                                                                │  │
│  │ ugv_base_bridge (UART Owner)                                   │  │
│  │  ├─ /cmd_vel ────────────────────────→ T=13 motors            │  │
│  │  ├─ /gimbal/setpoint ────────────────→ T=133 gimbal           │  │
│  │  └─ ESP32 feedback ──────────────────→ /ugv_base/feedback/*   │  │
│  │       ↕                                                        │  │
│  │  [Serial /dev/ttyTHS1 @ 115200] ←──→ ESP32 (ugv_base_ros)     │  │
│  │                                                                │  │
│  │ scan_mission_node (Action Server)                              │  │
│  │  ├─ /image_raw ─────────────────────→ JPEG frames             │  │
│  │  ├─ /gimbal/setpoint ←────────────── gimbal commands          │  │
│  │  └─ /scan_and_offload action ───────→ rsync to NAS            │  │
│  └────────────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────────┘
```

## 0) Prerequisites

### Disable Waveshare Auto-Run
Waveshare's default program grabs the camera + serial device. Disable it:
```bash
crontab -e  # Comment out @reboot line
sudo reboot
```

### Tailscale
Ensure Jetson + NAS are on the same tailnet and reachable via MagicDNS or Tailscale IP.

### NAS Setup (TerraMaster/Synology/etc.)
1. Enable SSH
2. Create user (e.g., `rover_ingest`)
3. Create destination directory: `/share/rover_ingest/ugv_rover_pt/missions/`
4. Add Jetson's SSH public key to user's `authorized_keys`

## 1) Configure

```bash
cp .env.example .env
# Edit .env with your settings
```

| Variable | Purpose | Example |
|----------|---------|---------|
| `UGV_SERIAL_PORT` | ESP32 UART device | `/dev/ttyTHS1` |
| `NAS_HOST` | NAS Tailscale hostname/IP | `terramaster` |
| `NAS_USER` | SSH username | `rover_ingest` |
| `NAS_DEST_DIR` | Destination path | `/share/rover_ingest/ugv_rover_pt/missions/` |

Find serial device on Jetson:
```bash
ls -la /dev/ttyTHS* /dev/ttyUSB*
```

## 2) Build & Run

```bash
# Build container
./scripts/build_container.sh

# Start system (detached)
./scripts/run_bringup.sh

# Enter container for debugging
./scripts/enter_container.sh
```

## 3) Trigger a Mission

From host:
```bash
./scripts/run_mission_once.sh
```

From container:
```bash
source /work/ros2_ws/install/setup.bash
ros2 action send_goal --feedback /scan_and_offload \
  ugv_rover_pt_interfaces/action/ScanAndOffload \
  "{duration_sec: 30.0, yaw_min_deg: -90.0, yaw_max_deg: 90.0, yaw_step_deg: 45.0, pitch_list_deg: [0.0, 30.0], frames_per_pose: 2, settle_ms: 500}"
```

## 4) Verify Results

- **Local**: `/data/missions/mission_<timestamp>/frames/*.jpg` + `meta.json`
- **NAS**: Folder appears under `NAS_DEST_DIR`

## 5) ROS 2 Packages

| Package | Description |
|---------|-------------|
| `ugv_base` | **ESP32 UART bridge** - handles motor control (T=13) and gimbal (T=133) |
| `ugv_rover_pt_bringup` | Launch orchestration - starts all nodes |
| `ugv_rover_pt_scan_mission` | Scan action server - coordinates capture + offload |
| `ugv_rover_pt_offload` | Rsync helper with retry logic and markers |
| `ugv_rover_pt_interfaces` | ROS 2 action/message definitions |

## 6) Testing

```bash
# Check ESP32 connection
ros2 service call /ugv_base/ping std_srvs/srv/Trigger

# Test gimbal
ros2 topic pub --once /gimbal/setpoint geometry_msgs/msg/Vector3 "{x: 45.0, y: 0.0, z: 0.0}"

# Monitor ESP32 feedback
ros2 topic echo /ugv_base/feedback/json

# View camera stream
ros2 topic hz /image_raw
```

## 7) Production Deployment

Install as systemd service:
```bash
sudo ./scripts/install_systemd.sh
```

## ESP32 Protocol Quick Reference

| Command | Type | Description |
|---------|------|-------------|
| `{"T":13,"X":0.5,"Z":0.3}` | Motor velocity | X=linear (m/s), Z=angular (rad/s) |
| `{"T":133,"X":45,"Y":0,"SPD":200,"ACC":10}` | Gimbal position | X=yaw°, Y=pitch°, SPD/ACC=servo params |
| `{"T":4,"cmd":2}` | Module select | 2=Gimbal (sent at startup) |
| `{"T":131,"cmd":1}` | Feedback flow | Enable sensor feedback stream |

See `CLAUDE.md` for complete protocol reference.

## Troubleshooting

### No Serial Response
```bash
ls -la /dev/ttyTHS*           # Check device exists
groups                         # Verify dialout group membership
ros2 service call /ugv_base/ping std_srvs/srv/Trigger  # Check stats
```

### Gimbal Not Moving
- Verify power to gimbal servos
- Check ugv_base TX count via ping service
- Confirm T=4 module selection sent at startup

### Camera Not Working
```bash
ls -la /dev/video*            # Check device
v4l2-ctl --list-devices       # List cameras
```

## License

MIT
