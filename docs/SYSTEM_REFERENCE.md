# UGV Rover PT Scan - Complete System Reference

## Table of Contents

1. [System Overview](#system-overview)
2. [Hardware Components](#hardware-components)
3. [Software Architecture](#software-architecture)
4. [Communication Protocols](#communication-protocols)
5. [ROS 2 Interface](#ros-2-interface)
6. [Available Commands](#available-commands)
7. [Mission System](#mission-system)
8. [Network Configuration](#network-configuration)
9. [File Locations](#file-locations)
10. [API Reference](#api-reference)

---

## System Overview

The UGV Rover PT Scan is a Waveshare UGV Rover PT platform powered by Jetson Orin, running ROS 2 Humble in Docker. It performs autonomous surveillance missions by sweeping a 2-axis gimbal, capturing frames, and offloading data to a NAS.

### System Specifications

| Component | Specification |
|-----------|---------------|
| Platform | Waveshare UGV Rover PT |
| Compute | NVIDIA Jetson Orin Nano/NX |
| OS | Ubuntu 22.04 (L4T) |
| Kernel | 5.15.136-tegra |
| ROS | ROS 2 Humble (Docker) |
| Controller | ESP32 (Waveshare firmware) |

### Current Deployment

| Parameter | Value |
|-----------|-------|
| Hostname | ubuntu |
| LAN IP | 192.168.1.44 |
| Tailscale IP | 100.88.142.110 |
| Project Path | /opt/ugv_rover_pt_scan |
| Mission Storage | /data/missions |

---

## Hardware Components

### 1. Camera

| Property | Value |
|----------|-------|
| Type | USB UVC Camera |
| Device | /dev/video0 |
| Resolution | 1280x720 |
| FPS | 30 (configurable) |
| Format | YUYV → RGB8 |
| Driver | v4l2_camera (ROS 2) |

**Capabilities:**
- Brightness, Contrast, Saturation, Hue controls
- Auto/Manual White Balance
- Auto/Manual Exposure
- Digital Pan/Tilt/Zoom

### 2. Gimbal (Pan-Tilt)

| Property | Value |
|----------|-------|
| Type | Waveshare 2-axis servo gimbal |
| Controller | ESP32 via UART |
| Yaw Range | -180° to +180° |
| Pitch Range | -30° to +90° |
| Protocol | JSON over serial |

### 3. Base (Drive Motors)

| Property | Value |
|----------|-------|
| Type | Tracked/wheeled differential drive |
| Controller | ESP32 via UART |
| Speed Range | -1.0 to +1.0 (float) |
| Control | Left/Right independent motors |

### 4. ESP32 Controller

| Property | Value |
|----------|-------|
| Serial Port | /dev/ttyTHS1 |
| Baud Rate | 115200 |
| Protocol | JSON + newline terminator |
| Functions | Gimbal, Motors, Lights, OLED, Feedback |

### 5. Lights

| Property | Value |
|----------|-------|
| Channels | 2 (IO4, IO5) |
| Type | PWM controlled |
| Range | 0-255 |

### 6. OLED Display

| Property | Value |
|----------|-------|
| Lines | 4 |
| Control | Via ESP32 (T=3 command) |

---

## Software Architecture

### Docker Container Stack

```
┌─────────────────────────────────────────────────────────────────┐
│                    Host: Jetson Orin (Ubuntu 22.04)             │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │              Docker Container: ugv_ros                     │  │
│  │  Base Image: ros:humble-ros-base                          │  │
│  │                                                           │  │
│  │  ┌─────────────────────────────────────────────────────┐  │  │
│  │  │              ROS 2 Humble Runtime                    │  │  │
│  │  │                                                     │  │  │
│  │  │  Nodes:                                             │  │  │
│  │  │  ├── /v4l2_camera_node (camera driver)              │  │  │
│  │  │  ├── /gimbal_node (gimbal serial control)           │  │  │
│  │  │  └── /scan_mission_node (action server)             │  │  │
│  │  │                                                     │  │  │
│  │  │  Topics:                                            │  │  │
│  │  │  ├── /image_raw (sensor_msgs/Image)                 │  │  │
│  │  │  ├── /camera_info (sensor_msgs/CameraInfo)          │  │  │
│  │  │  └── /gimbal/setpoint (geometry_msgs/Vector3)       │  │  │
│  │  │                                                     │  │  │
│  │  │  Actions:                                           │  │  │
│  │  │  └── /scan_and_offload (ScanAndOffload)             │  │  │
│  │  └─────────────────────────────────────────────────────┘  │  │
│  │                                                           │  │
│  │  Mounts:                                                  │  │
│  │  ├── /work/ros2_ws ← ros2_ws (source code)               │  │
│  │  ├── /data/missions ← /data/missions (mission storage)   │  │
│  │  ├── /dev ← /dev (device access)                         │  │
│  │  └── /root/.ssh ← ~/.ssh (SSH keys for offload)          │  │
│  └───────────────────────────────────────────────────────────┘  │
│                                                                 │
│  Systemd Service: ugv-rover-pt.service                         │
│  └── Starts Docker Compose on boot                             │
└─────────────────────────────────────────────────────────────────┘
```

### ROS 2 Packages

| Package | Type | Description |
|---------|------|-------------|
| ugv_rover_pt_bringup | CMake | Launch orchestration |
| ugv_rover_pt_gimbal_serial | Python | Gimbal serial driver |
| ugv_rover_pt_scan_mission | Python | Scan action server |
| ugv_rover_pt_offload | Python | rsync offload utility |
| ugv_rover_pt_interfaces | CMake | Action/message definitions |

---

## Communication Protocols

### ESP32 Serial Protocol

**Connection:** `/dev/ttyTHS1` @ 115200 baud

**Format:** JSON object + newline (`\n`)

```python
# Example
msg = json.dumps({"T": 133, "X": 45, "Y": 0, "SPD": 200, "ACC": 10}) + '\n'
serial.write(msg.encode('utf-8'))
```

### Required Initialization Sequence

**CRITICAL:** Must send before gimbal/motor commands work:

```python
# 1. Set feedback interval (50ms)
{"T": 142, "cmd": 50}

# 2. Enable serial feedback
{"T": 131, "cmd": 1}

# 3. Disable echo
{"T": 143, "cmd": 0}

# 4. Select module type (2 = Gimbal/PT)
{"T": 4, "cmd": 2}

# 5. Set platform version (optional but recommended)
{"T": 900, "main": 2, "module": 2}
# main: 1=RaspRover, 2=UGV Rover, 3=UGV Beast
# module: 0=None, 1=RoArm, 2=PT Gimbal
```

### Command Reference Table

| Command | T | Parameters | Description |
|---------|---|------------|-------------|
| Emergency Stop | 0 | - | `{"T":0}` |
| Base Motors | 1 | L, R (float -1.0 to 1.0) | `{"T":1,"L":0.5,"R":0.5}` |
| OLED Display | 3 | lineNum (0-3), Text | `{"T":3,"lineNum":0,"Text":"Hello"}` |
| Module Select | 4 | cmd (0/1/2) | `{"T":4,"cmd":2}` |
| Lights | 132 | IO4, IO5 (0-255) | `{"T":132,"IO4":255,"IO5":255}` |
| Gimbal Control | 133 | X, Y, SPD, ACC | `{"T":133,"X":45,"Y":30,"SPD":200,"ACC":10}` |
| Gimbal Base | 141 | X, Y, SPD | `{"T":141,"X":45,"Y":30,"SPD":200}` |
| Feedback Interval | 142 | cmd (ms) | `{"T":142,"cmd":50}` |
| Echo Control | 143 | cmd (0/1) | `{"T":143,"cmd":0}` |
| Platform Version | 900 | main, module | `{"T":900,"main":2,"module":2}` |

### Gimbal Control Details (T=133)

```json
{"T": 133, "X": yaw, "Y": pitch, "SPD": speed, "ACC": acceleration}
```

| Parameter | Type | Range | Description |
|-----------|------|-------|-------------|
| X | float | -180 to +180 | Yaw angle (degrees) |
| Y | float | -30 to +90 | Pitch angle (degrees) |
| SPD | int | 0-255 | Movement speed (0=default) |
| ACC | int | 0-255 | Acceleration (0=default) |

**Recommended:** SPD=200, ACC=10

### Motor Control Details (T=1)

```json
{"T": 1, "L": left_speed, "R": right_speed}
```

| Parameter | Type | Range | Description |
|-----------|------|-------|-------------|
| L | float | -1.0 to +1.0 | Left motor (negative=reverse) |
| R | float | -1.0 to +1.0 | Right motor (negative=reverse) |

**Movement Examples:**
- Forward: `{"T":1,"L":0.5,"R":0.5}`
- Backward: `{"T":1,"L":-0.5,"R":-0.5}`
- Turn Left: `{"T":1,"L":-0.3,"R":0.3}`
- Turn Right: `{"T":1,"L":0.3,"R":-0.3}`
- Stop: `{"T":0}` or `{"T":1,"L":0,"R":0}`

---

## ROS 2 Interface

### Nodes

| Node | Package | Description |
|------|---------|-------------|
| /v4l2_camera_node | v4l2_camera | USB camera driver |
| /gimbal_node | ugv_rover_pt_gimbal_serial | Gimbal control |
| /scan_mission_node | ugv_rover_pt_scan_mission | Mission action server |

### Topics

| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| /image_raw | sensor_msgs/Image | v4l2_camera_node | Raw camera frames |
| /camera_info | sensor_msgs/CameraInfo | v4l2_camera_node | Camera calibration |
| /gimbal/setpoint | geometry_msgs/Vector3 | - | Gimbal target (x=yaw, y=pitch) |

### Actions

| Action | Type | Server | Description |
|--------|------|--------|-------------|
| /scan_and_offload | ScanAndOffload | scan_mission_node | Execute scan mission |

### ScanAndOffload Action Definition

```yaml
# Goal
float32 duration_sec      # Max mission duration
float32 yaw_min_deg       # Minimum yaw angle
float32 yaw_max_deg       # Maximum yaw angle
float32 yaw_step_deg      # Yaw increment
float32[] pitch_list_deg  # List of pitch angles
int32 frames_per_pose     # Frames to capture at each pose
int32 settle_ms           # Settle time after gimbal move

# Feedback
string state              # WAIT_IMAGE, SCAN, PACKAGE, OFFLOAD
float32 progress          # 0.0 to 1.0

# Result
bool success              # Mission success
string local_bundle       # Local path to mission data
string nas_bundle         # NAS path (if offloaded)
string message            # Status/error message
```

---

## Available Commands

### Direct Serial Control (Python)

```python
import serial
import json
import time

ser = serial.Serial('/dev/ttyTHS1', 115200, timeout=0.2)

def send_cmd(cmd):
    msg = (json.dumps(cmd) + '\n').encode('utf-8')
    ser.write(msg)
    ser.flush()

# Initialize
send_cmd({"T":142,"cmd":50})
send_cmd({"T":131,"cmd":1})
send_cmd({"T":143,"cmd":0})
send_cmd({"T":4,"cmd":2})
time.sleep(0.5)

# Gimbal: look right 45°, up 30°
send_cmd({"T":133,"X":45,"Y":30,"SPD":200,"ACC":10})

# Motors: move forward
send_cmd({"T":1,"L":0.4,"R":0.4})
time.sleep(1)
send_cmd({"T":0})  # stop

# Lights: on
send_cmd({"T":132,"IO4":255,"IO5":255})
time.sleep(2)
send_cmd({"T":132,"IO4":0,"IO5":0})  # off

# OLED: display text
send_cmd({"T":3,"lineNum":0,"Text":"Hello UGV"})

ser.close()
```

### ROS 2 Commands (from container)

```bash
# Enter container
docker exec -it ugv_ros bash
source /work/ros2_ws/install/setup.bash

# List nodes
ros2 node list

# List topics
ros2 topic list

# View camera feed info
ros2 topic hz /image_raw
ros2 topic echo /image_raw --once

# Control gimbal
ros2 topic pub --once /gimbal/setpoint geometry_msgs/msg/Vector3 "{x: 45.0, y: 30.0, z: 0.0}"

# Center gimbal
ros2 topic pub --once /gimbal/setpoint geometry_msgs/msg/Vector3 "{x: 0.0, y: 0.0, z: 0.0}"

# Run scan mission
ros2 action send_goal /scan_and_offload ugv_rover_pt_interfaces/action/ScanAndOffload \
  "{duration_sec: 60.0, yaw_min_deg: -90.0, yaw_max_deg: 90.0, yaw_step_deg: 30.0, pitch_list_deg: [0.0, 30.0], frames_per_pose: 2, settle_ms: 500}" \
  --feedback
```

### SSH Remote Commands

```bash
# From remote machine
SSH_TARGET="jetson@100.88.142.110"

# Check system status
ssh $SSH_TARGET "systemctl status ugv-rover-pt.service"
ssh $SSH_TARGET "docker ps"

# Gimbal control via ROS
ssh $SSH_TARGET "docker exec ugv_ros bash -lc 'source /work/ros2_ws/install/setup.bash && ros2 topic pub --once /gimbal/setpoint geometry_msgs/msg/Vector3 \"{x: 30.0, y: 0.0, z: 0.0}\"'"

# Direct serial control
ssh $SSH_TARGET 'python3 << "EOF"
import serial, json, time
ser = serial.Serial("/dev/ttyTHS1", 115200, timeout=0.2)
def send(c): ser.write((json.dumps(c)+"\n").encode()); ser.flush()
send({"T":133,"X":30,"Y":0,"SPD":200,"ACC":10})
ser.close()
EOF'

# Move backward
ssh $SSH_TARGET 'python3 << "EOF"
import serial, json, time
ser = serial.Serial("/dev/ttyTHS1", 115200, timeout=0.2)
def send(c): ser.write((json.dumps(c)+"\n").encode()); ser.flush()
send({"T":1,"L":-0.4,"R":-0.4})
time.sleep(1)
send({"T":0})
ser.close()
EOF'
```

---

## Mission System

### Mission Data Structure

```
/data/missions/
├── mission_2026-01-19T18-51-23/
│   ├── meta.json
│   └── frames/
│       ├── yaw_-090.0_pitch_+00.0_i_000000_k_0.jpg
│       ├── yaw_-090.0_pitch_+00.0_i_000001_k_1.jpg
│       ├── yaw_-090.0_pitch_+30.0_i_000002_k_0.jpg
│       └── ...
├── mission_2026-01-19T18-51-23.uploading  # During upload
└── mission_2026-01-19T18-51-23.complete   # After successful upload
```

### Frame Filename Format

```
yaw_{yaw:+06.1f}_pitch_{pitch:+05.1f}_i_{index:06d}_k_{frame_num}.jpg
```

Example: `yaw_+045.0_pitch_+30.0_i_000014_k_1.jpg`
- Yaw: +45.0°
- Pitch: +30.0°
- Index: 14 (sequential)
- Frame: 1 (second frame at this pose)

### meta.json Structure

```json
{
  "timestamp": "2026-01-19T18-51-23",
  "duration_sec": 30.0,
  "yaw": {
    "min": -90.0,
    "max": 90.0,
    "step": 30.0
  },
  "pitch_list": [0.0, 30.0],
  "frames_per_pose": 2,
  "settle_ms": 500,
  "frames_saved": 28,
  "nas": {
    "host": "192.168.1.20",
    "user": "boxhead",
    "dest_dir": "/share/boxhead/ugv_rover_pt/missions/"
  }
}
```

### Offload Markers

| Marker File | Meaning |
|-------------|---------|
| `.uploading` | Upload in progress |
| `.complete` | Upload successful |
| (none) | Not yet attempted or failed |

---

## Network Configuration

### Interfaces

| Interface | IP | Purpose |
|-----------|-----|---------|
| eth0 | 192.168.1.44 | Local LAN |
| tailscale0 | 100.88.142.110 | VPN (Tailscale) |

### NAS Configuration

| Parameter | Value |
|-----------|-------|
| Hostname | box-nas |
| Tailscale IP | 100.74.45.35 |
| LAN IP | 192.168.1.20 |
| SSH User | boxhead |
| Destination | /share/boxhead/ugv_rover_pt/missions/ |

### SSH Key Location

- Host: `/home/jetson/.ssh/id_ed25519`
- Container: `/root/.ssh/id_ed25519` (mounted)

---

## File Locations

### On Jetson

| Path | Description |
|------|-------------|
| /opt/ugv_rover_pt_scan/ | Project root |
| /opt/ugv_rover_pt_scan/.env | Environment configuration |
| /opt/ugv_rover_pt_scan/docker/ | Docker files |
| /opt/ugv_rover_pt_scan/ros2_ws/ | ROS 2 workspace |
| /data/missions/ | Mission data storage |
| /etc/systemd/system/ugv-rover-pt.service | Systemd service |
| /home/jetson/.ssh/ | SSH keys |

### In Container

| Path | Description |
|------|-------------|
| /work/ros2_ws/ | ROS 2 workspace (mounted) |
| /work/ros2_ws/install/ | Built packages |
| /data/missions/ | Mission storage (mounted) |
| /root/.ssh/ | SSH keys (mounted) |

### Environment Variables (.env)

```bash
ROS_DOMAIN_ID=0
UGV_SERIAL_PORT=/dev/ttyTHS1
UGV_SERIAL_BAUD=115200
UGV_CAMERA_DEVICE=/dev/video0
UGV_CAMERA_FPS=30
UGV_CAMERA_WIDTH=1280
UGV_CAMERA_HEIGHT=720
UGV_MISSION_ROOT=/data/missions
NAS_HOST=192.168.1.20
NAS_USER=boxhead
NAS_DEST_DIR=/share/boxhead/ugv_rover_pt/missions/
```

---

## API Reference

### Python Control Library

```python
class UGVController:
    """Complete UGV control interface."""

    def __init__(self, port='/dev/ttyTHS1', baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self._initialize()

    def _send(self, cmd):
        self.ser.write((json.dumps(cmd) + '\n').encode())
        self.ser.flush()

    def _initialize(self):
        """Required initialization sequence."""
        self._send({"T":142,"cmd":50})
        time.sleep(0.1)
        self._send({"T":131,"cmd":1})
        time.sleep(0.1)
        self._send({"T":143,"cmd":0})
        time.sleep(0.1)
        self._send({"T":4,"cmd":2})
        time.sleep(0.5)

    # Gimbal
    def gimbal(self, yaw, pitch, speed=200, accel=10):
        """Move gimbal to position."""
        self._send({"T":133,"X":yaw,"Y":pitch,"SPD":speed,"ACC":accel})

    def gimbal_center(self):
        """Center gimbal."""
        self.gimbal(0, 0)

    # Motors
    def drive(self, left, right):
        """Set motor speeds (-1.0 to 1.0)."""
        self._send({"T":1,"L":left,"R":right})

    def forward(self, speed=0.4):
        self.drive(speed, speed)

    def backward(self, speed=0.4):
        self.drive(-speed, -speed)

    def turn_left(self, speed=0.3):
        self.drive(-speed, speed)

    def turn_right(self, speed=0.3):
        self.drive(speed, -speed)

    def stop(self):
        self._send({"T":0})

    # Lights
    def lights(self, base=0, head=0):
        """Set light brightness (0-255)."""
        self._send({"T":132,"IO4":base,"IO5":head})

    def lights_on(self):
        self.lights(255, 255)

    def lights_off(self):
        self.lights(0, 0)

    # Display
    def oled(self, line, text):
        """Display text on OLED (line 0-3)."""
        self._send({"T":3,"lineNum":line,"Text":text})

    def close(self):
        self.ser.close()
```

### Usage Example

```python
ugv = UGVController()

# Look around
ugv.gimbal(45, 0)
time.sleep(1)
ugv.gimbal(-45, 0)
time.sleep(1)
ugv.gimbal_center()

# Move
ugv.forward()
time.sleep(1)
ugv.stop()

# Lights
ugv.lights_on()
time.sleep(2)
ugv.lights_off()

# Display
ugv.oled(0, "Mission Start")

ugv.close()
```

---

## Troubleshooting

### Common Issues

| Issue | Cause | Solution |
|-------|-------|----------|
| Gimbal not moving | Missing init sequence | Send T=4,cmd=2 first |
| Motors not moving | Wrong speed type | Use floats -1.0 to 1.0 |
| Camera busy | Waveshare app.py running | `pkill -f app.py` |
| Serial busy | Multiple processes | `fuser -v /dev/ttyTHS1` |
| Offload fails | SSH not configured | Check SSH keys |
| Container won't start | Docker not ready | Wait for network |

### Diagnostic Commands

```bash
# Check service
systemctl status ugv-rover-pt.service

# Check container
docker ps
docker logs ugv_ros

# Check devices
ls -l /dev/video* /dev/ttyTHS*
fuser -v /dev/video0 /dev/ttyTHS1

# Check ROS
docker exec ugv_ros bash -lc "source /work/ros2_ws/install/setup.bash && ros2 node list"

# Test serial
python3 -c "import serial; s=serial.Serial('/dev/ttyTHS1',115200); print('OK'); s.close()"
```

---

## Version History

| Date | Version | Changes |
|------|---------|---------|
| 2026-01-20 | 1.0.0 | Initial provisioning complete |
| 2026-01-20 | 1.0.1 | Fixed gimbal init sequence (T=4,cmd=2) |
| 2026-01-20 | 1.0.2 | Fixed motor speed type (floats not ints) |
