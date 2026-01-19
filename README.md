# UGV Rover PT Jetson Orin — Scan + Offload Mission (ROS 2 Humble)

Boots into **IDLE** (camera + gimbal ready). On command, it:
1) sweeps gimbal to scan surroundings
2) saves frames to `/data/missions/mission_<timestamp>/`
3) offloads bundle to TerraMaster over Tailscale via `rsync+ssh`
4) returns to IDLE

## 0) One-time prerequisites

### Disable Waveshare auto-run main program
Waveshare's default program often grabs the camera + serial device. Disable it (typically via `crontab -e` and commenting the `@reboot` line), then reboot.

### Tailscale
Ensure Jetson + TerraMaster are logged into the same tailnet and the NAS is reachable (MagicDNS or Tailscale IP).

### TerraMaster
Recommended: enable **SSH**, create a user (e.g. `rover_ingest`), and ensure the destination directory exists:
`/share/rover_ingest/ugv_rover_pt/box-rover/`

Add the Jetson's SSH key to the NAS user's `authorized_keys`.

## 1) Configure

Copy `.env.example` to `.env` and edit:
- `UGV_SERIAL_PORT` (ESP32 UART device)
- `NAS_HOST`, `NAS_USER`, `NAS_DEST_DIR`

Find serial device on Jetson host:
```bash
ls -l /dev/ttyTHS* /dev/ttyUSB* /dev/ttyACM*
```

## 2) Run with Docker Compose

```bash
cd docker
cp ../.env.example ../.env
# edit ../.env

docker compose up -d --build
```

Enter the container:
```bash
../scripts/enter_container.sh
```

## 3) Trigger a mission

From host:
```bash
./scripts/run_mission_once.sh
```

From inside container:
```bash
source /work/ros2_ws/install/setup.bash
ros2 action send_goal /scan_and_offload ugv_rover_pt_scan_mission/action/ScanAndOffload "{duration_sec: 30.0}"
```

## 4) What to look for

- Local: `/data/missions/mission_<timestamp>/frames/*.jpg` + `meta.json`
- NAS: folder appears under `NAS_DEST_DIR`

## 5) Packages

- `ugv_rover_pt_bringup`: launch camera + gimbal serial + mission server
- `ugv_rover_pt_gimbal_serial`: UART JSON gimbal control (T=133)
- `ugv_rover_pt_scan_mission`: action server (scan → save → offload → idle)
- `ugv_rover_pt_offload`: rsync helper with retries/markers
