# ugv_base — Unified ESP32 UART Bridge for Waveshare UGV Rover

ROS 2 Humble package that serves as the **sole UART owner** for the Waveshare UGV Rover ESP32, handling both base motor control and gimbal commands through a single serial connection.

## Overview

The `ugv_base` package consolidates all ESP32 communication into one node:
- **Motor control** via `/cmd_vel` (Twist) → `T=13` JSON commands
- **Gimbal control** via `/gimbal/setpoint` (Vector3) → `T=133` JSON commands
- **Feedback publishing** from ESP32 → `/ugv_base/feedback/json`
- **Health monitoring** via `/ugv_base/ping` service and diagnostics

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      ugv_base_bridge Node                       │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │  Subscribers:                                             │  │
│  │    /cmd_vel (Twist)        → TX scheduler @ 20Hz          │  │
│  │    /gimbal/setpoint (Vec3) → Immediate TX                 │  │
│  │                                                           │  │
│  │  Publishers:                                              │  │
│  │    /ugv_base/feedback/json     (String)                   │  │
│  │    /ugv_base/feedback/connected (Bool)                    │  │
│  │    /ugv_base/diagnostics       (DiagnosticArray)          │  │
│  │                                                           │  │
│  │  Services:                                                │  │
│  │    /ugv_base/ping (std_srvs/Trigger)                      │  │
│  └───────────────────────────────────────────────────────────┘  │
│                              │                                  │
│                              ▼                                  │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │              Serial Port (/dev/ttyTHS1)                   │  │
│  │                                                           │  │
│  │  TX Thread: JSON commands (T=4, T=13, T=131, T=133, etc.) │  │
│  │  RX Thread: JSON feedback (T=1001, T=1005) → Queue        │  │
│  └───────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
                    ┌─────────────────┐
                    │   ESP32 MCU     │
                    │  (ugv_base_ros) │
                    └─────────────────┘
```

## ESP32 JSON Protocol Reference

### Commands Sent (TX)

| Type | Command | Description | Example |
|------|---------|-------------|---------|
| `T=4` | Module Select | Select module at startup (2=Gimbal PT) | `{"T":4,"cmd":2}` |
| `T=13` | CMD_ROS_CTRL | Velocity command (m/s, rad/s) | `{"T":13,"X":0.5,"Z":0.3}` |
| `T=131` | Feedback Flow | Enable/disable feedback stream | `{"T":131,"cmd":1}` |
| `T=133` | Gimbal Simple | Set gimbal position with speed | `{"T":133,"X":45,"Y":0,"SPD":200,"ACC":10}` |
| `T=136` | Heartbeat | Set heartbeat timeout (ms) | `{"T":136,"cmd":1000}` |
| `T=142` | Feedback Interval | Set feedback rate (ms) | `{"T":142,"cmd":100}` |
| `T=143` | UART Echo | Enable/disable echo mode | `{"T":143,"cmd":0}` |

### Feedback Received (RX)

| Type | Description | Key Fields |
|------|-------------|------------|
| `T=1001` | Full sensor feedback | `L`, `R` (motor), `ax/ay/az` (accel), `gx/gy/gz` (gyro), `mx/my/mz` (mag), `odl/odr` (odom), `v` (voltage), `pan/tilt` (IMU angles) |
| `T=1005` | Command acknowledgment | `id`, `status` (0=success) |

### Gimbal Ranges

| Axis | Range | Notes |
|------|-------|-------|
| Yaw (X) | -180° to +180° | Pan axis |
| Pitch (Y) | -30° to +90° | Tilt axis |
| Speed (SPD) | 0-255 | Servo speed |
| Acceleration (ACC) | 0-255 | Servo acceleration |

## Configuration (params.yaml)

```yaml
ugv_base_bridge:
  ros__parameters:
    serial:
      device: "/dev/ttyTHS1"      # GPIO UART on Jetson
      baud: 115200
      read_timeout_ms: 50
      write_timeout_ms: 50
      reconnect_interval_ms: 1000

    uart_echo_mode:
      enabled: false              # Disable ESP32 echo

    heartbeat:
      timeout_ms: 1000            # ESP32 heartbeat watchdog

    feedback_flow:
      enabled: true               # Enable feedback stream
      interval_ms: 100            # Feedback rate
      request_once_on_startup: false

    ros_ctrl:
      topic: "/cmd_vel"
      send_rate_hz: 20.0          # TX rate for velocity commands
      cmd_timeout_ms: 300         # Stale command timeout
      send_zero_on_timeout: true  # Send zero velocity if stale
      clamp_x: 1.0                # Max linear velocity (m/s)
      clamp_z: 5.0                # Max angular velocity (rad/s)
      scale_x: 1.0
      scale_z: 1.0

    gimbal:
      topic: "/gimbal/setpoint"
      default_spd: 200
      default_acc: 10
      clamp_yaw_deg: 180.0
      clamp_pitch_min_deg: -30.0
      clamp_pitch_max_deg: 90.0

    health:
      max_feedback_age_ms: 500    # Max age before "disconnected"
```

## Usage

### Launch with Bringup

The package is launched automatically by `bringup.launch.py`:

```bash
ros2 launch ugv_rover_pt_bringup bringup.launch.py
```

### Standalone Testing

```bash
# Run the node directly
ros2 run ugv_base bridge_node --ros-args -p serial.device:=/dev/ttyTHS1

# Check health
ros2 service call /ugv_base/ping std_srvs/srv/Trigger

# Move gimbal
ros2 topic pub --once /gimbal/setpoint geometry_msgs/msg/Vector3 "{x: 45.0, y: 0.0, z: 0.0}"

# Send velocity command
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"

# Monitor feedback
ros2 topic echo /ugv_base/feedback/json
```

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Subscribe | Velocity commands |
| `/gimbal/setpoint` | `geometry_msgs/Vector3` | Subscribe | Gimbal position (x=yaw, y=pitch) |
| `/ugv_base/feedback/json` | `std_msgs/String` | Publish | Raw JSON from ESP32 |
| `/ugv_base/feedback/connected` | `std_msgs/Bool` | Publish | Connection status |
| `/ugv_base/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Publish | Health diagnostics |

## Services

| Service | Type | Description |
|---------|------|-------------|
| `/ugv_base/ping` | `std_srvs/Trigger` | Health check with stats |

## Startup Sequence

1. Open serial port
2. Send module selection: `{"T":4,"cmd":2}` (gimbal mode)
3. Configure UART echo: `{"T":143,"cmd":0}` (disabled)
4. Set heartbeat timeout: `{"T":136,"cmd":1000}`
5. Enable feedback flow: `{"T":131,"cmd":1}`
6. Set feedback interval: `{"T":142,"cmd":100}`
7. Start RX thread and TX scheduler

## Thread Safety

- **RX Thread**: Reads serial in background, parses JSON, queues for main thread
- **TX Scheduler**: Timer-based, runs in main executor thread
- **Serial Lock**: Protects write operations
- **Queue**: Thread-safe handoff from RX to publisher

## Dependencies

- `rclpy`
- `geometry_msgs`
- `std_msgs`
- `std_srvs`
- `diagnostic_msgs`
- `python3-serial` (pyserial)

## Hardware Notes

### Jetson Orin Serial Ports

| Device | Description |
|--------|-------------|
| `/dev/ttyTHS0` | UART0 (often debug console) |
| `/dev/ttyTHS1` | UART1 (ESP32 connection on UGV Rover PT) |
| `/dev/ttyTHS2` | UART2 (available) |

### Waveshare UGV Rover PT Kit

- ESP32 running `ugv_base_ros` firmware
- 2-DOF gimbal with SCServos
- D500 LiDAR (separate driver)
- OAK-D Lite depth camera (separate driver)
- 3S LiPo with UPS module

## Troubleshooting

### No Serial Response

1. Check device exists: `ls -la /dev/ttyTHS*`
2. Check permissions: user must be in `dialout` group
3. Verify baud rate matches ESP32 (115200)

### Gimbal Not Moving

1. Verify T=133 commands are being sent (check TX count in ping)
2. Confirm module selection was sent (T=4 cmd=2)
3. Check gimbal power supply

### Feedback Not Received

1. Check RX count in ping service
2. Verify feedback flow enabled (T=131 cmd=1)
3. Look for parse errors in diagnostics
