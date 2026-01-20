# LLM AI Engineer - UGV Rover PT System Test Prompt

## Context

You are an AI Engineer with SSH access to a Waveshare UGV Rover PT robot running on a Jetson Orin. The system runs ROS 2 Humble in Docker and has various hardware capabilities. Your task is to systematically test all available resources and verify the system is fully operational.

## Connection Details

```
SSH Target: jetson@100.88.142.110
Serial Port: /dev/ttyTHS1 @ 115200 baud
Protocol: JSON + newline terminator
```

## System Capabilities to Test

### 1. Gimbal (Pan-Tilt Camera Mount)

**Hardware:** 2-axis servo gimbal controlled via ESP32

**Capabilities:**
- Yaw (horizontal): -180° to +180°
- Pitch (vertical): -30° to +90°
- Variable speed and acceleration

**Test Commands:**
```bash
# Via ROS 2 topic
ssh jetson@100.88.142.110 "docker exec ugv_ros bash -lc 'source /work/ros2_ws/install/setup.bash && ros2 topic pub --once /gimbal/setpoint geometry_msgs/msg/Vector3 \"{x: YAW, y: PITCH, z: 0.0}\"'"

# Via direct serial (Python)
ssh jetson@100.88.142.110 'python3 << "EOF"
import serial, json, time
ser = serial.Serial("/dev/ttyTHS1", 115200, timeout=0.2)
def send(c): ser.write((json.dumps(c)+"\n").encode()); ser.flush()
# Initialize (required once per session)
send({"T":142,"cmd":50}); time.sleep(0.1)
send({"T":131,"cmd":1}); time.sleep(0.1)
send({"T":143,"cmd":0}); time.sleep(0.1)
send({"T":4,"cmd":2}); time.sleep(0.5)
# Move gimbal
send({"T":133,"X":YAW,"Y":PITCH,"SPD":200,"ACC":10})
ser.close()
EOF'
```

**Test Sequence:**
1. Center gimbal (0°, 0°)
2. Pan right (+45°, 0°)
3. Pan left (-45°, 0°)
4. Tilt up (0°, +30°)
5. Tilt down (0°, -15°)
6. Combined movement (+30°, +20°)
7. Return to center

---

### 2. Base Motors (Differential Drive)

**Hardware:** Tracked/wheeled base with left and right motors

**Capabilities:**
- Independent left/right motor control
- Speed range: -1.0 (full reverse) to +1.0 (full forward)
- **IMPORTANT:** Speed values must be FLOATS, not integers

**Test Commands:**
```bash
ssh jetson@100.88.142.110 'python3 << "EOF"
import serial, json, time
ser = serial.Serial("/dev/ttyTHS1", 115200, timeout=0.2)
def send(c): ser.write((json.dumps(c)+"\n").encode()); ser.flush()
# Move (L=left motor, R=right motor, floats -1.0 to 1.0)
send({"T":1,"L":LEFT_SPEED,"R":RIGHT_SPEED})
time.sleep(DURATION)
send({"T":0})  # Emergency stop
ser.close()
EOF'
```

**Test Sequence:**
1. Forward (L=0.3, R=0.3) for 0.5s
2. Backward (L=-0.3, R=-0.3) for 0.5s
3. Turn left (L=-0.2, R=0.2) for 0.3s
4. Turn right (L=0.2, R=-0.2) for 0.3s
5. Stop (T=0)

**Safety:** Use low speeds (0.2-0.4) and short durations for testing.

---

### 3. Lights

**Hardware:** 2 PWM-controlled light channels (IO4, IO5)

**Capabilities:**
- Brightness: 0 (off) to 255 (max)
- Independent control of base lights and head lights

**Test Commands:**
```bash
ssh jetson@100.88.142.110 'python3 << "EOF"
import serial, json, time
ser = serial.Serial("/dev/ttyTHS1", 115200, timeout=0.2)
def send(c): ser.write((json.dumps(c)+"\n").encode()); ser.flush()
# Lights (IO4=base, IO5=head, range 0-255)
send({"T":132,"IO4":BRIGHTNESS1,"IO5":BRIGHTNESS2})
ser.close()
EOF'
```

**Test Sequence:**
1. Both lights full on (255, 255) - hold 2s
2. Base only (255, 0) - hold 1s
3. Head only (0, 255) - hold 1s
4. Dim both (128, 128) - hold 1s
5. Lights off (0, 0)

---

### 4. Camera

**Hardware:** USB UVC camera at /dev/video0

**Capabilities:**
- Resolution: 1280x720
- FPS: 30
- ROS 2 topic: /image_raw

**Test Commands:**
```bash
# Check camera is publishing
ssh jetson@100.88.142.110 "docker exec ugv_ros bash -lc 'source /work/ros2_ws/install/setup.bash && ros2 topic hz /image_raw --window 10'"

# Capture a single frame (check it's receiving)
ssh jetson@100.88.142.110 "docker exec ugv_ros bash -lc 'source /work/ros2_ws/install/setup.bash && ros2 topic echo /image_raw --once | head -20'"

# List camera info
ssh jetson@100.88.142.110 "v4l2-ctl --device=/dev/video0 --all | head -50"
```

**Test Sequence:**
1. Verify camera device exists
2. Check ROS topic is publishing
3. Verify frame rate (~8-30 fps)
4. Confirm resolution

---

### 5. OLED Display

**Hardware:** 4-line OLED display on ESP32

**Capabilities:**
- 4 lines of text (line 0-3)
- Controlled via T=3 command

**Test Commands:**
```bash
ssh jetson@100.88.142.110 'python3 << "EOF"
import serial, json
ser = serial.Serial("/dev/ttyTHS1", 115200, timeout=0.2)
def send(c): ser.write((json.dumps(c)+"\n").encode()); ser.flush()
# Display text (lineNum 0-3)
send({"T":3,"lineNum":LINE_NUM,"Text":"YOUR_TEXT"})
ser.close()
EOF'
```

**Test Sequence:**
1. Line 0: "UGV Test Mode"
2. Line 1: "Line 2 Test"
3. Line 2: "Line 3 Test"
4. Line 3: Current timestamp

---

### 6. Scan Mission (ROS 2 Action)

**Hardware:** Coordinated gimbal + camera capture

**Capabilities:**
- Automated gimbal sweep pattern
- Frame capture at each pose
- Local storage with metadata
- Optional NAS offload

**Test Command:**
```bash
ssh jetson@100.88.142.110 "docker exec ugv_ros bash -lc 'source /work/ros2_ws/install/setup.bash && ros2 action send_goal /scan_and_offload ugv_rover_pt_interfaces/action/ScanAndOffload \"{duration_sec: 30.0, yaw_min_deg: -45.0, yaw_max_deg: 45.0, yaw_step_deg: 45.0, pitch_list_deg: [0.0], frames_per_pose: 1, settle_ms: 500}\" --feedback'"
```

**Test Sequence:**
1. Run minimal scan (small yaw range, single pitch)
2. Verify frames captured
3. Check meta.json created
4. Verify gimbal moved during scan

---

### 7. System Health

**Test Commands:**
```bash
# Service status
ssh jetson@100.88.142.110 "systemctl status ugv-rover-pt.service"

# Container status
ssh jetson@100.88.142.110 "docker ps"

# ROS nodes
ssh jetson@100.88.142.110 "docker exec ugv_ros bash -lc 'source /work/ros2_ws/install/setup.bash && ros2 node list'"

# ROS topics
ssh jetson@100.88.142.110 "docker exec ugv_ros bash -lc 'source /work/ros2_ws/install/setup.bash && ros2 topic list'"

# Disk space
ssh jetson@100.88.142.110 "df -h /data"

# Memory
ssh jetson@100.88.142.110 "free -h"

# Serial port available
ssh jetson@100.88.142.110 "ls -l /dev/ttyTHS1"
```

---

## Complete Test Script

Execute all tests in sequence:

```bash
#!/bin/bash
SSH="ssh jetson@100.88.142.110"

echo "=== UGV ROVER PT SYSTEM TEST ==="
echo ""

echo "1. System Health..."
$SSH "systemctl is-active ugv-rover-pt.service"
$SSH "docker ps --format '{{.Names}}: {{.Status}}'"

echo ""
echo "2. ROS Nodes..."
$SSH "docker exec ugv_ros bash -lc 'source /work/ros2_ws/install/setup.bash && ros2 node list'"

echo ""
echo "3. Camera Test..."
$SSH "docker exec ugv_ros bash -lc 'source /work/ros2_ws/install/setup.bash && timeout 5 ros2 topic hz /image_raw --window 5'" 2>/dev/null || echo "Camera publishing"

echo ""
echo "4. Gimbal Test..."
$SSH 'python3 << "EOF"
import serial, json, time
ser = serial.Serial("/dev/ttyTHS1", 115200, timeout=0.2)
def send(c): ser.write((json.dumps(c)+"\n").encode()); ser.flush(); print(f"Sent: {c}")
send({"T":142,"cmd":50}); time.sleep(0.1)
send({"T":131,"cmd":1}); time.sleep(0.1)
send({"T":143,"cmd":0}); time.sleep(0.1)
send({"T":4,"cmd":2}); time.sleep(0.5)
print("Gimbal: Center")
send({"T":133,"X":0,"Y":0,"SPD":200,"ACC":10}); time.sleep(1)
print("Gimbal: Right 30")
send({"T":133,"X":30,"Y":0,"SPD":200,"ACC":10}); time.sleep(1)
print("Gimbal: Left 30")
send({"T":133,"X":-30,"Y":0,"SPD":200,"ACC":10}); time.sleep(1)
print("Gimbal: Center")
send({"T":133,"X":0,"Y":0,"SPD":200,"ACC":10})
ser.close()
print("Gimbal test complete")
EOF'

echo ""
echo "5. Lights Test..."
$SSH 'python3 << "EOF"
import serial, json, time
ser = serial.Serial("/dev/ttyTHS1", 115200, timeout=0.2)
def send(c): ser.write((json.dumps(c)+"\n").encode()); ser.flush()
print("Lights ON")
send({"T":132,"IO4":255,"IO5":255})
time.sleep(2)
print("Lights OFF")
send({"T":132,"IO4":0,"IO5":0})
ser.close()
print("Lights test complete")
EOF'

echo ""
echo "6. Motors Test (small movement)..."
$SSH 'python3 << "EOF"
import serial, json, time
ser = serial.Serial("/dev/ttyTHS1", 115200, timeout=0.2)
def send(c): ser.write((json.dumps(c)+"\n").encode()); ser.flush()
print("Forward briefly")
send({"T":1,"L":0.2,"R":0.2})
time.sleep(0.3)
send({"T":0})
time.sleep(0.5)
print("Backward briefly")
send({"T":1,"L":-0.2,"R":-0.2})
time.sleep(0.3)
send({"T":0})
ser.close()
print("Motors test complete")
EOF'

echo ""
echo "7. OLED Test..."
$SSH 'python3 << "EOF"
import serial, json, time
ser = serial.Serial("/dev/ttyTHS1", 115200, timeout=0.2)
def send(c): ser.write((json.dumps(c)+"\n").encode()); ser.flush()
send({"T":3,"lineNum":0,"Text":"System Test OK"})
time.sleep(0.1)
send({"T":3,"lineNum":1,"Text":"All Tests Pass"})
ser.close()
print("OLED test complete")
EOF'

echo ""
echo "=== ALL TESTS COMPLETE ==="
```

---

## Expected Results Checklist

| Test | Expected Result | Pass/Fail |
|------|-----------------|-----------|
| Service Active | `active` | |
| Container Running | `ugv_ros: Up` | |
| ROS Nodes | 3 nodes listed | |
| Camera Publishing | ~8-30 fps | |
| Gimbal Movement | Physical movement observed | |
| Lights | Visible on/off | |
| Motors | Small forward/back movement | |
| OLED | Text displayed | |

---

## Safety Notes

1. **Motors:** Always use low speeds (≤0.4) and short durations (≤1s) for testing
2. **Stop Command:** `{"T":0}` is emergency stop - use if anything goes wrong
3. **Clear Area:** Ensure robot has space to move during motor tests
4. **Supervision:** Physical observation recommended during movement tests

---

## Troubleshooting

| Issue | Check | Fix |
|-------|-------|-----|
| SSH timeout | Network | Use Tailscale IP 100.88.142.110 |
| Gimbal no response | Init sequence | Send T=4,cmd=2 first |
| Motors no response | Speed type | Use floats (-1.0 to 1.0) |
| Camera not publishing | Device busy | Kill conflicting processes |
| Container not running | Docker | `docker compose up -d` |

---

## Report Format

After testing, provide a summary:

```
## UGV System Test Report

**Date:** YYYY-MM-DD HH:MM
**Tester:** [AI/Human]

### Results

| Component | Status | Notes |
|-----------|--------|-------|
| System Service | ✓/✗ | |
| Docker Container | ✓/✗ | |
| ROS Nodes | ✓/✗ | |
| Camera | ✓/✗ | FPS: X |
| Gimbal | ✓/✗ | |
| Motors | ✓/✗ | |
| Lights | ✓/✗ | |
| OLED | ✓/✗ | |

### Issues Found
- [List any issues]

### Recommendations
- [List any recommendations]
```
