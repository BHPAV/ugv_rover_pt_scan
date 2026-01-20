#!/usr/bin/env python3
"""
ugv_probe.py — ROS + system probe for Waveshare UGV Rover (Jetson)
- Auto-discovers "surfaces" you should have access to
- Flags missing/unknown surfaces
- Outputs JSON report + human summary

Usage:
  python3 ugv_probe.py --out report.json
  python3 ugv_probe.py --hz-seconds 5 --out report.json
"""

# ----------------------------------------
# 1. Imports + tiny helpers
# ----------------------------------------
# Minimal deps: stdlib only
import argparse
import json
import os
import platform
import re
import shutil
import socket
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


def which(cmd: str) -> Optional[str]:
    return shutil.which(cmd)


def run(cmd: List[str], timeout: int = 15) -> Tuple[int, str, str]:
    """Run a command safely. Returns (rc, stdout, stderr)."""
    try:
        p = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
        return p.returncode, (p.stdout or "").strip(), (p.stderr or "").strip()
    except FileNotFoundError:
        return 127, "", f"not found: {cmd[0]}"
    except subprocess.TimeoutExpired:
        return 124, "", f"timeout after {timeout}s: {' '.join(cmd)}"
    except Exception as e:
        return 1, "", f"error: {e}"


def read_text(path: str, max_bytes: int = 200_000) -> Optional[str]:
    try:
        p = Path(path)
        if not p.exists():
            return None
        data = p.read_bytes()
        if len(data) > max_bytes:
            data = data[:max_bytes]
        return data.decode("utf-8", errors="replace").strip()
    except Exception:
        return None


def list_dir(path: str) -> List[str]:
    try:
        return sorted([str(p) for p in Path(path).iterdir()])
    except Exception:
        return []


def exists(path: str) -> bool:
    try:
        return Path(path).exists()
    except Exception:
        return False


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def add_surface(report: Dict[str, Any], name: str, status: str, notes: str = "", data: Any = None) -> None:
    report["surfaces"][name] = {
        "status": status,  # PASS / WARN / FAIL / UNKNOWN
        "notes": notes,
        "data": data,
    }


def is_root() -> bool:
    return hasattr(os, "geteuid") and os.geteuid() == 0


# ----------------------------------------
# 2. System probes (hardware/OS)
# ----------------------------------------
def probe_basic_system(report: Dict[str, Any]) -> None:
    add_surface(report, "system.hostname", "PASS", data=socket.gethostname())
    add_surface(report, "system.platform", "PASS", data={
        "python": sys.version.split()[0],
        "os": platform.platform(),
        "machine": platform.machine(),
        "kernel": platform.release(),
    })

    model = read_text("/proc/device-tree/model") or read_text("/sys/firmware/devicetree/base/model")
    if model:
        add_surface(report, "system.device_model", "PASS", data=model)
    else:
        add_surface(report, "system.device_model", "UNKNOWN", "could not read device-tree model")

    uptime = read_text("/proc/uptime")
    if uptime:
        secs = float(uptime.split()[0])
        add_surface(report, "system.uptime_seconds", "PASS", data=secs)
    else:
        add_surface(report, "system.uptime_seconds", "UNKNOWN", "could not read /proc/uptime")


def probe_resources(report: Dict[str, Any]) -> None:
    # Disk
    rc, out, err = run(["df", "-h", "/"])
    if rc == 0 and out:
        add_surface(report, "storage.root_df", "PASS", data=out)
    else:
        add_surface(report, "storage.root_df", "UNKNOWN", err or "df failed")

    # Memory
    mem = read_text("/proc/meminfo")
    if mem:
        add_surface(report, "memory.meminfo", "PASS", data=mem.splitlines()[:12])
    else:
        add_surface(report, "memory.meminfo", "UNKNOWN", "could not read /proc/meminfo")

    # Swap / pressure
    psi_mem = read_text("/proc/pressure/memory")
    if psi_mem:
        add_surface(report, "memory.psi", "PASS", data=psi_mem)
    else:
        add_surface(report, "memory.psi", "UNKNOWN", "psi not available")


def probe_jetson_telemetry(report: Dict[str, Any]) -> None:
    # tegrastats presence; we won't run it continuously, just check availability
    ts = which("tegrastats")
    if ts:
        add_surface(report, "jetson.tegrastats", "PASS", f"found at {ts}")
    else:
        add_surface(report, "jetson.tegrastats", "WARN", "tegrastats not found (not fatal, but useful)")

    # nvpmodel and jetson_clocks can exist on some builds
    for cmd in ["nvpmodel", "jetson_clocks"]:
        p = which(cmd)
        add_surface(report, f"jetson.{cmd}", "PASS" if p else "UNKNOWN",
                    notes=f"found at {p}" if p else f"{cmd} not found")


def probe_thermal(report: Dict[str, Any]) -> None:
    zones = []
    base = Path("/sys/class/thermal")
    if base.exists():
        for z in sorted(base.glob("thermal_zone*")):
            ztype = read_text(str(z / "type"))
            ztemp = read_text(str(z / "temp"))
            zones.append({"path": str(z), "type": ztype, "temp_mC": int(ztemp) if ztemp and ztemp.isdigit() else ztemp})
    if zones:
        add_surface(report, "thermal.zones", "PASS", data=zones)
    else:
        add_surface(report, "thermal.zones", "UNKNOWN", "no thermal zones found or not accessible")


def probe_watchdog_rtc(report: Dict[str, Any]) -> None:
    # Watchdog
    if exists("/dev/watchdog") or exists("/dev/watchdog0"):
        add_surface(report, "safety.watchdog", "PASS", "watchdog device present",
                    data=[p for p in ["/dev/watchdog", "/dev/watchdog0"] if exists(p)])
    else:
        add_surface(report, "safety.watchdog", "WARN", "no /dev/watchdog* device found (consider enabling for field reliability)")

    # RTC
    rtc_paths = list(Path("/dev").glob("rtc*"))
    if rtc_paths:
        add_surface(report, "time.rtc", "PASS", data=[str(p) for p in rtc_paths])
    else:
        add_surface(report, "time.rtc", "WARN", "no /dev/rtc* found (timekeeping offline may be weak)")


def probe_i2c(report: Dict[str, Any]) -> None:
    i2c_devs = [str(p) for p in Path("/dev").glob("i2c-*")]
    if i2c_devs:
        add_surface(report, "bus.i2c_devices", "PASS", data=i2c_devs)
    else:
        add_surface(report, "bus.i2c_devices", "UNKNOWN", "no /dev/i2c-* found (may be normal if disabled or container lacks /dev)")

    i2c_tools = which("i2cdetect")
    add_surface(report, "bus.i2c_tools", "PASS" if i2c_tools else "UNKNOWN",
                notes=f"found at {i2c_tools}" if i2c_tools else "i2c-tools not installed")


def probe_v4l2(report: Dict[str, Any]) -> None:
    videos = [str(p) for p in Path("/dev").glob("video*")]
    if videos:
        add_surface(report, "camera.v4l2_devices", "PASS", data=videos)
    else:
        add_surface(report, "camera.v4l2_devices", "FAIL", "no /dev/video* devices found")

    v4l2ctl = which("v4l2-ctl")
    if not v4l2ctl:
        add_surface(report, "camera.v4l2_ctl", "UNKNOWN", "v4l2-ctl not installed (optional)")
        return

    # Enumerate devices
    rc, out, err = run(["v4l2-ctl", "--list-devices"], timeout=10)
    if rc == 0 and out:
        add_surface(report, "camera.v4l2_list_devices", "PASS", data=out)
    else:
        add_surface(report, "camera.v4l2_list_devices", "UNKNOWN", err or "v4l2-ctl --list-devices failed")


def probe_serial(report: Dict[str, Any]) -> None:
    # Useful for gimbal/motor controller MCU lines
    ttys = []
    for pat in ["ttyTHS*", "ttyUSB*", "ttyACM*"]:
        ttys.extend([str(p) for p in Path("/dev").glob(pat)])
    if ttys:
        add_surface(report, "io.serial_ports", "PASS", data=sorted(set(ttys)))
    else:
        add_surface(report, "io.serial_ports", "WARN", "no common serial ports found (container may hide /dev)")


def probe_services_and_containers(report: Dict[str, Any]) -> None:
    # systemd (host)
    systemctl = which("systemctl")
    if systemctl:
        rc, out, err = run(["systemctl", "is-system-running"], timeout=5)
        add_surface(report, "systemd.health", "PASS" if rc == 0 else "WARN", data=out or err)
    else:
        add_surface(report, "systemd.health", "UNKNOWN", "systemctl not available (container or non-systemd OS)")

    # docker
    docker = which("docker")
    if docker:
        rc, out, err = run(["docker", "ps", "--format", "{{.Names}}\t{{.Status}}"], timeout=8)
        if rc == 0:
            add_surface(report, "docker.ps", "PASS", data=out.splitlines()[:50])
        else:
            add_surface(report, "docker.ps", "WARN", err or "docker ps failed")
    else:
        add_surface(report, "docker.ps", "UNKNOWN", "docker CLI not found")


def probe_network(report: Dict[str, Any]) -> None:
    # Interfaces + IPs
    rc, out, err = run(["ip", "-o", "addr"], timeout=8)
    if rc == 0 and out:
        add_surface(report, "net.ip_addr", "PASS", data=out.splitlines()[:200])
    else:
        add_surface(report, "net.ip_addr", "UNKNOWN", err or "ip addr failed")

    # Routing
    rc, out, err = run(["ip", "route"], timeout=8)
    if rc == 0 and out:
        add_surface(report, "net.route", "PASS", data=out.splitlines()[:100])
    else:
        add_surface(report, "net.route", "UNKNOWN", err or "ip route failed")

    # Tailscale
    ts = which("tailscale")
    if ts:
        rc, out, err = run(["tailscale", "status", "--json"], timeout=10)
        add_surface(report, "net.tailscale", "PASS" if rc == 0 else "WARN", data=out if out else err)
    else:
        add_surface(report, "net.tailscale", "UNKNOWN", "tailscale CLI not found")

    # Time sync (systemd-timesyncd / chrony)
    timedatectl = which("timedatectl")
    if timedatectl:
        rc, out, err = run(["timedatectl", "show", "-p", "NTPSynchronized", "-p", "TimeUSec", "-p", "Timezone"], timeout=6)
        add_surface(report, "time.sync", "PASS" if rc == 0 else "UNKNOWN", data=out if out else err)
    else:
        add_surface(report, "time.sync", "UNKNOWN", "timedatectl not found")


# ----------------------------------------
# 3. ROS 2 probes (graph + health)
# ----------------------------------------
def ros2_available() -> bool:
    return which("ros2") is not None


def ros2_list(report: Dict[str, Any], what: str) -> Tuple[str, List[str]]:
    rc, out, err = run(["ros2", what, "list"], timeout=12)
    if rc == 0:
        lines = [ln.strip() for ln in out.splitlines() if ln.strip()]
        return "PASS", lines
    return "WARN", [err or f"ros2 {what} list failed"]


def ros2_params_for_nodes(nodes: List[str], limit_nodes: int = 25) -> Dict[str, Any]:
    results: Dict[str, Any] = {}
    for n in nodes[:limit_nodes]:
        rc, out, err = run(["ros2", "param", "list", n], timeout=10)
        if rc == 0:
            params = [ln.strip() for ln in out.splitlines() if ln.strip()]
            results[n] = {"params": params[:200]}
        else:
            results[n] = {"error": err or "param list failed"}
    if len(nodes) > limit_nodes:
        results["_truncated"] = f"only first {limit_nodes} nodes"
    return results


def ros2_topic_hz(topic: str, seconds: int) -> Dict[str, Any]:
    # ros2 topic hz runs until interrupted; use timeout and parse last lines if any
    rc, out, err = run(["ros2", "topic", "hz", topic], timeout=max(3, seconds))
    payload = {"rc": rc, "stdout": out, "stderr": err}
    # Best-effort parse "average rate: X"
    m = re.search(r"average rate:\s*([0-9.]+)", out)
    if m:
        payload["average_hz"] = float(m.group(1))
    return payload


def probe_ros(report: Dict[str, Any], hz_seconds: int) -> None:
    if not ros2_available():
        add_surface(report, "ros2.available", "FAIL", "ros2 CLI not found in PATH (source ROS setup or run inside container)")
        return

    add_surface(report, "ros2.available", "PASS", "ros2 CLI found")

    # Nodes / topics / services / actions
    s, nodes = ros2_list(report, "node")
    add_surface(report, "ros2.nodes", s, data=nodes)

    s, topics = ros2_list(report, "topic")
    add_surface(report, "ros2.topics", s, data=topics)

    s, services = ros2_list(report, "service")
    add_surface(report, "ros2.services", s, data=services)

    s, actions = ros2_list(report, "action")
    add_surface(report, "ros2.actions", s, data=actions)

    # Params (sample)
    if isinstance(nodes, list) and nodes and s != "WARN":
        add_surface(report, "ros2.params_sample", "PASS", data=ros2_params_for_nodes(nodes))

    # Heuristic: find likely camera image topics and measure hz
    image_topics = [t for t in topics if re.search(r"(image|camera).*(raw|rgb|color)?", t, re.IGNORECASE)]
    image_topics = image_topics[:5]  # cap
    hz_results = {}
    for t in image_topics:
        hz_results[t] = ros2_topic_hz(t, hz_seconds)
        # Tiny pause so we don't hammer ROS graph
        time.sleep(0.2)

    if hz_results:
        add_surface(report, "ros2.image_topic_hz", "PASS", data=hz_results)
    else:
        add_surface(report, "ros2.image_topic_hz", "UNKNOWN", "no obvious image topics found (or ros2 topic list empty)")


# ----------------------------------------
# 4. Battery/power heuristics (best-effort)
# ----------------------------------------
def probe_power_battery(report: Dict[str, Any]) -> None:
    # Linux power_supply is the standard place if exposed
    ps = Path("/sys/class/power_supply")
    if ps.exists():
        entries = []
        for dev in sorted(ps.iterdir()):
            d = {"name": dev.name}
            for key in ["type", "status", "capacity", "voltage_now", "current_now", "power_now", "online"]:
                v = read_text(str(dev / key))
                if v is not None:
                    d[key] = v
            entries.append(d)
        if entries:
            add_surface(report, "power.power_supply", "PASS", data=entries)
            # Flag if battery exists but missing capacity/voltage
            has_bat = any(e.get("type", "").lower() == "battery" for e in entries)
            if has_bat:
                missing = []
                for e in entries:
                    if e.get("type", "").lower() == "battery":
                        if "capacity" not in e:
                            missing.append("capacity")
                        if "voltage_now" not in e:
                            missing.append("voltage_now")
                if missing:
                    add_surface(report, "power.battery_telemetry_quality", "WARN",
                                f"battery present but missing fields: {sorted(set(missing))}")
                else:
                    add_surface(report, "power.battery_telemetry_quality", "PASS", "battery telemetry looks populated")
            else:
                add_surface(report, "power.battery_telemetry_quality", "WARN", "no Battery-type power_supply found")
            return

    add_surface(report, "power.power_supply", "UNKNOWN", "no /sys/class/power_supply (or inaccessible)")
    add_surface(report, "power.battery_telemetry_quality", "WARN", "battery telemetry not discoverable via power_supply")


# ----------------------------------------
# 5. Main + summary rendering
# ----------------------------------------
def summarize(report: Dict[str, Any]) -> str:
    # Order by severity: FAIL, WARN, UNKNOWN, PASS
    buckets = {"FAIL": [], "WARN": [], "UNKNOWN": [], "PASS": []}
    for k, v in report["surfaces"].items():
        buckets.get(v["status"], buckets["UNKNOWN"]).append(k)

    def fmt(names: List[str]) -> str:
        return "\n".join([f"  - {n}: {report['surfaces'][n].get('notes','')}".rstrip() for n in names])

    lines = []
    lines.append(f"UGV Probe Summary @ {report['meta']['created_at_utc']}")
    lines.append(f"Target: {report['meta'].get('target','(local)')}")
    lines.append("")
    for level in ["FAIL", "WARN", "UNKNOWN", "PASS"]:
        if buckets[level]:
            lines.append(f"{level} ({len(buckets[level])})")
            lines.append(fmt(sorted(buckets[level])))
            lines.append("")
    return "\n".join(lines).strip()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="ugv_probe_report.json", help="output JSON filename")
    ap.add_argument("--target", default="local", help="informational target label (e.g., jetson@100.x)")
    ap.add_argument("--hz-seconds", type=int, default=5, help="timeout seconds for ros2 topic hz checks")
    args = ap.parse_args()

    report: Dict[str, Any] = {
        "meta": {
            "created_at_utc": now_iso(),
            "target": args.target,
            "is_root": is_root(),
        },
        "surfaces": {},
    }

    # System probes
    probe_basic_system(report)
    probe_resources(report)
    probe_jetson_telemetry(report)
    probe_thermal(report)
    probe_watchdog_rtc(report)
    probe_i2c(report)
    probe_v4l2(report)
    probe_serial(report)
    probe_services_and_containers(report)
    probe_network(report)
    probe_power_battery(report)

    # ROS probes
    probe_ros(report, hz_seconds=args.hz_seconds)

    # Write JSON
    out_path = Path(args.out).expanduser().resolve()
    out_path.write_text(json.dumps(report, indent=2, sort_keys=True))
    print(summarize(report))
    print("")
    print(f"Wrote JSON report: {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
