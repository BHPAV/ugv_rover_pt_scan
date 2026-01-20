# ----------------------------------------
# bridge_node.py
# ----------------------------------------
# Summary:
#   ROS 2 <-> Waveshare UGV Rover UART JSON bridge.
#   - TX scheduler sends CMD_ROS_CTRL (T=13) at fixed rate from /cmd_vel
#   - Configures feedback flow (T=131, T=142, optional T=130)
#   - Configures heartbeat (T=136)
#   - Optional echo control (T=143)
#   - Safe RX thread reads newline-delimited JSON and enqueues for main-thread publishing
#   - /ugv_base/ping (std_srvs/Trigger) reports health deterministically

from __future__ import annotations

import json
import queue
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

try:
    import serial  # pyserial
    from serial import SerialException
except Exception as e:  # pragma: no cover
    serial = None
    SerialException = Exception


# ----------------------------------------
# Small utilities
# ----------------------------------------
# Summary: helpers for time, clamping, and compact newline-delimited JSON encoding.

def _now_mono() -> float:
    return time.monotonic()

def _ms_to_s(ms: int) -> float:
    return max(0.0, float(ms) / 1000.0)

def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))

def _compact_json_line(obj: Dict[str, Any]) -> bytes:
    # Waveshare examples commonly use readline(); newline-delimited JSON keeps things simple.
    return (json.dumps(obj, separators=(",", ":")) + "\n").encode("utf-8")


# ----------------------------------------
# Protocol encoders (exact Waveshare subset)
# ----------------------------------------
# Summary: byte payload builders for the commands we need.

def cmd_ros_ctrl(x_mps: float, z_rps: float) -> bytes:
    # CMD_ROS_CTRL
    return _compact_json_line({"T": 13, "X": float(x_mps), "Z": float(z_rps)})

def cmd_base_feedback_once() -> bytes:
    return _compact_json_line({"T": 130})

def cmd_base_feedback_flow(enabled: bool) -> bytes:
    return _compact_json_line({"T": 131, "cmd": 1 if enabled else 0})

def cmd_feedback_flow_interval(extra_ms: int) -> bytes:
    return _compact_json_line({"T": 142, "cmd": int(extra_ms)})

def cmd_uart_echo(enabled: bool) -> bytes:
    return _compact_json_line({"T": 143, "cmd": 1 if enabled else 0})

def cmd_heartbeat_set(timeout_ms: int) -> bytes:
    return _compact_json_line({"T": 136, "cmd": int(timeout_ms)})

def cmd_gimbal_simple(yaw_deg: float, pitch_deg: float, spd: int, acc: int) -> bytes:
    # CMD_GIMBAL_CTRL_SIMPLE
    return _compact_json_line(
        {"T": 133, "X": float(yaw_deg), "Y": float(pitch_deg), "SPD": int(spd), "ACC": int(acc)}
    )

def cmd_module_select(module: int) -> bytes:
    # Module selection: 0=None, 1=RoArm-M2, 2=Gimbal (PT)
    return _compact_json_line({"T": 4, "cmd": int(module)})


# ----------------------------------------
# Parameter groups (params.yaml-compatible)
# ----------------------------------------
# Summary: dataclasses mirroring nested YAML structure.

@dataclass(frozen=True)
class SerialParams:
    device: str
    baud: int
    read_timeout_ms: int
    write_timeout_ms: int
    reconnect_interval_ms: int

@dataclass(frozen=True)
class UartEchoParams:
    enabled: bool

@dataclass(frozen=True)
class HeartbeatParams:
    timeout_ms: int

@dataclass(frozen=True)
class FeedbackFlowParams:
    enabled: bool
    interval_ms: int
    request_once_on_startup: bool

@dataclass(frozen=True)
class RosCtrlParams:
    topic: str
    send_rate_hz: float
    cmd_timeout_ms: int
    send_zero_on_timeout: bool
    clamp_x: float
    clamp_z: float
    scale_x: float
    scale_z: float

@dataclass(frozen=True)
class GimbalParams:
    topic: str
    default_spd: int
    default_acc: int
    clamp_yaw_deg: float
    clamp_pitch_min_deg: float
    clamp_pitch_max_deg: float

@dataclass(frozen=True)
class HealthParams:
    max_feedback_age_ms: int

@dataclass(frozen=True)
class Params:
    serial: SerialParams
    uart_echo_mode: UartEchoParams
    heartbeat: HeartbeatParams
    feedback_flow: FeedbackFlowParams
    ros_ctrl: RosCtrlParams
    gimbal: GimbalParams
    health: HealthParams


# ----------------------------------------
# Main bridge node
# ----------------------------------------
# Summary:
#   Owns the UART device, runs RX thread, schedules TX, and exposes health endpoints.

class UgvBaseBridge(Node):
    def __init__(self) -> None:
        super().__init__("ugv_base_bridge")

        if serial is None:
            raise RuntimeError(
                "pyserial is not available. Install python3-serial (apt) or pyserial (pip)."
            )

        self.p = self._load_params()

        # ---- ROS I/O ----
        self.pub_raw = self.create_publisher(String, "/ugv_base/feedback/json", 10)
        self.pub_connected = self.create_publisher(Bool, "/ugv_base/feedback/connected", 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, "/ugv_base/diagnostics", 10)

        self.sub_cmd = self.create_subscription(Twist, self.p.ros_ctrl.topic, self._on_cmd_vel, 10)

        # Gimbal control is optional; keep it enabled but allow topic="" to disable.
        if self.p.gimbal.topic.strip():
            self.sub_gimbal = self.create_subscription(
                Vector3, self.p.gimbal.topic, self._on_gimbal_setpoint, 10
            )
        else:
            self.sub_gimbal = None

        self.srv_ping = self.create_service(Trigger, "/ugv_base/ping", self._on_ping)

        # ---- Serial state ----
        self._ser: Optional[serial.Serial] = None
        self._ser_lock = threading.Lock()

        # RX thread feeds this queue; main thread drains + publishes
        self._rx_q: "queue.Queue[Tuple[str, Dict[str, Any]]]" = queue.Queue(maxsize=500)

        self._rx_stop = threading.Event()
        self._rx_thread: Optional[threading.Thread] = None

        # ---- Telemetry ----
        self._last_rx_mono: float = 0.0
        self._last_tx_mono: float = 0.0
        self._last_connected_pub: Optional[bool] = None

        self._rx_count: int = 0
        self._tx_count: int = 0
        self._parse_errors: int = 0
        self._rx_dropped: int = 0
        self._serial_errors: int = 0

        # ---- Latest cmd_vel ----
        self._cmd_x: float = 0.0
        self._cmd_z: float = 0.0
        self._last_cmd_mono: float = 0.0

        # ---- Throttle logging ----
        self._last_warn_mono: float = 0.0
        self._warn_period_s: float = 2.0

        # ---- Timers ----
        # Drain RX queue frequently; keep ROS publishing in the main thread.
        self._rx_drain_timer = self.create_timer(0.02, self._drain_rx_queue)

        # TX scheduler for CMD_ROS_CTRL
        hz = max(1.0, float(self.p.ros_ctrl.send_rate_hz))
        self._tx_timer = self.create_timer(1.0 / hz, self._tx_tick)

        # Diagnostics/connected state at low rate
        self._diag_timer = self.create_timer(1.0, self._publish_diagnostics)

        # Kick off initial connect
        self._ensure_connected(initial=True)

        self.get_logger().info(
            "ugv_base_bridge started. "
            f"serial={self.p.serial.device}@{self.p.serial.baud} "
            f"cmd_vel_topic={self.p.ros_ctrl.topic} "
            f"gimbal_topic={self.p.gimbal.topic or '(disabled)'}"
        )

    # ----------------------------------------
    # Parameter parsing (params.yaml-compatible)
    # ----------------------------------------
    # Summary: declare + read dot-names that map to nested YAML keys.

    def _load_params(self) -> Params:
        def p(name: str, default: Any) -> Any:
            self.declare_parameter(name, default)
            return self.get_parameter(name).value

        serial_p = SerialParams(
            device=str(p("serial.device", "/dev/ttyTHS0")),
            baud=int(p("serial.baud", 115200)),
            read_timeout_ms=int(p("serial.read_timeout_ms", 50)),
            write_timeout_ms=int(p("serial.write_timeout_ms", 50)),
            reconnect_interval_ms=int(p("serial.reconnect_interval_ms", 1000)),
        )

        uart_echo_p = UartEchoParams(
            enabled=bool(p("uart_echo_mode.enabled", False))
        )

        heartbeat_p = HeartbeatParams(
            timeout_ms=int(p("heartbeat.timeout_ms", 1000))
        )

        feedback_p = FeedbackFlowParams(
            enabled=bool(p("feedback_flow.enabled", True)),
            interval_ms=int(p("feedback_flow.interval_ms", 100)),
            request_once_on_startup=bool(p("feedback_flow.request_once_on_startup", False)),
        )

        ros_ctrl_p = RosCtrlParams(
            topic=str(p("ros_ctrl.topic", "/cmd_vel")),
            send_rate_hz=float(p("ros_ctrl.send_rate_hz", 20.0)),
            cmd_timeout_ms=int(p("ros_ctrl.cmd_timeout_ms", 300)),
            send_zero_on_timeout=bool(p("ros_ctrl.send_zero_on_timeout", True)),
            clamp_x=float(p("ros_ctrl.clamp_x", 1.0)),
            clamp_z=float(p("ros_ctrl.clamp_z", 5.0)),
            scale_x=float(p("ros_ctrl.scale_x", 1.0)),
            scale_z=float(p("ros_ctrl.scale_z", 1.0)),
        )

        gimbal_p = GimbalParams(
            topic=str(p("gimbal.topic", "/gimbal/setpoint")),
            default_spd=int(p("gimbal.default_spd", 0)),
            default_acc=int(p("gimbal.default_acc", 0)),
            clamp_yaw_deg=float(p("gimbal.clamp_yaw_deg", 180.0)),
            clamp_pitch_min_deg=float(p("gimbal.clamp_pitch_min_deg", -30.0)),
            clamp_pitch_max_deg=float(p("gimbal.clamp_pitch_max_deg", 90.0)),
        )

        health_p = HealthParams(
            max_feedback_age_ms=int(p("health.max_feedback_age_ms", 500))
        )

        return Params(
            serial=serial_p,
            uart_echo_mode=uart_echo_p,
            heartbeat=heartbeat_p,
            feedback_flow=feedback_p,
            ros_ctrl=ros_ctrl_p,
            gimbal=gimbal_p,
            health=health_p,
        )

    # ----------------------------------------
    # Serial connection lifecycle
    # ----------------------------------------
    # Summary: connect, configure device, run RX thread, and handle reconnect.

    def _ensure_connected(self, initial: bool = False) -> None:
        if self._ser is not None and getattr(self._ser, "is_open", False):
            return

        # Respect reconnect interval in steady-state
        if not initial:
            # No busy-loop; timer tick already controls cadence.
            pass

        try:
            self._open_serial()
            self._configure_device()
            self._start_rx_thread()
        except Exception as e:
            self._serial_errors += 1
            self._close_serial()
            self._warn_throttled(f"Serial connect/config failed: {e}")

    def _open_serial(self) -> None:
        with self._ser_lock:
            if self._ser is not None and getattr(self._ser, "is_open", False):
                return

            self.get_logger().info(f"Opening serial {self.p.serial.device} @ {self.p.serial.baud}")
            self._ser = serial.Serial(
                port=self.p.serial.device,
                baudrate=self.p.serial.baud,
                timeout=_ms_to_s(self.p.serial.read_timeout_ms),
                write_timeout=_ms_to_s(self.p.serial.write_timeout_ms),
            )

    def _close_serial(self) -> None:
        with self._ser_lock:
            try:
                if self._ser is not None:
                    if getattr(self._ser, "is_open", False):
                        self._ser.close()
            except Exception:
                pass
            self._ser = None

    def _configure_device(self) -> None:
        # Keep this short; small sleeps can help on some UART stacks.
        # All commands are exact Waveshare JSON forms.
        cfg_cmds = [
            ("module_select", cmd_module_select(2)),  # Select gimbal (PT) module first
            ("echo", cmd_uart_echo(self.p.uart_echo_mode.enabled)),
            ("heartbeat", cmd_heartbeat_set(self.p.heartbeat.timeout_ms)),
            ("feedback_flow", cmd_base_feedback_flow(self.p.feedback_flow.enabled)),
            ("feedback_interval", cmd_feedback_flow_interval(self.p.feedback_flow.interval_ms)),
        ]
        if self.p.feedback_flow.request_once_on_startup:
            cfg_cmds.append(("feedback_once", cmd_base_feedback_once()))

        for name, payload in cfg_cmds:
            ok = self._write_serial(payload)
            if not ok:
                raise RuntimeError(f"Failed to send config cmd: {name}")
            time.sleep(0.05)

    def _start_rx_thread(self) -> None:
        if self._rx_thread is not None and self._rx_thread.is_alive():
            return

        self._rx_stop.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, name="ugv_base_rx", daemon=True)
        self._rx_thread.start()

    # ----------------------------------------
    # RX loop (thread) -> queue (main thread publishes)
    # ----------------------------------------
    # Summary: read newline-delimited JSON, parse safely, enqueue (raw, dict).

    def _rx_loop(self) -> None:
        while not self._rx_stop.is_set():
            ser = self._ser
            if ser is None or not getattr(ser, "is_open", False):
                time.sleep(0.2)
                continue

            try:
                line: bytes = ser.readline()
                if not line:
                    continue

                raw = line.decode("utf-8", errors="replace").strip()
                if not raw:
                    continue

                obj = json.loads(raw)
                if not isinstance(obj, dict):
                    self._parse_errors += 1
                    continue

                self._rx_count += 1
                self._last_rx_mono = _now_mono()

                # Enqueue without blocking; drop if overloaded.
                try:
                    self._rx_q.put_nowait((raw, obj))
                except queue.Full:
                    self._rx_dropped += 1

            except (json.JSONDecodeError, ValueError):
                self._parse_errors += 1
                continue
            except SerialException as e:
                self._serial_errors += 1
                self._warn_throttled(f"Serial RX error: {e}")
                self._close_serial()
                time.sleep(_ms_to_s(self.p.serial.reconnect_interval_ms))
            except Exception as e:
                self._serial_errors += 1
                self._warn_throttled(f"Unexpected RX error: {e}")
                time.sleep(0.2)

    def _drain_rx_queue(self) -> None:
        # Publish up to N messages per tick to prevent starvation.
        max_per_tick = 50
        n = 0

        while n < max_per_tick:
            try:
                raw, _obj = self._rx_q.get_nowait()
            except queue.Empty:
                break

            msg = String()
            msg.data = raw
            self.pub_raw.publish(msg)
            n += 1

        # Update connected publisher based on last_rx
        self._publish_connected()

    # ----------------------------------------
    # TX path (timer + callbacks)
    # ----------------------------------------
    # Summary: send CMD_ROS_CTRL at fixed rate; send gimbal commands on demand.

    def _tx_tick(self) -> None:
        # Ensure serial is up (reconnect if needed)
        if self._ser is None or not getattr(self._ser, "is_open", False):
            self._ensure_connected(initial=False)
            return

        age_ms = int((_now_mono() - self._last_cmd_mono) * 1000.0) if self._last_cmd_mono > 0 else 10**9
        stale = age_ms > self.p.ros_ctrl.cmd_timeout_ms

        if stale and not self.p.ros_ctrl.send_zero_on_timeout:
            # Intentionally send nothing (may trip heartbeat if configured too aggressively).
            return

        # If stale, send zero; else send latest.
        x = 0.0 if stale else self._cmd_x
        z = 0.0 if stale else self._cmd_z

        payload = cmd_ros_ctrl(x_mps=x, z_rps=z)
        self._write_serial(payload)

    def _on_cmd_vel(self, msg: Twist) -> None:
        # Scale then clamp (Waveshare ranges are typically X in [-1,1], Z in [-5,5]).
        x = float(msg.linear.x) * self.p.ros_ctrl.scale_x
        z = float(msg.angular.z) * self.p.ros_ctrl.scale_z

        x = _clamp(x, -self.p.ros_ctrl.clamp_x, self.p.ros_ctrl.clamp_x)
        z = _clamp(z, -self.p.ros_ctrl.clamp_z, self.p.ros_ctrl.clamp_z)

        self._cmd_x = x
        self._cmd_z = z
        self._last_cmd_mono = _now_mono()

    def _on_gimbal_setpoint(self, msg: Vector3) -> None:
        # Interpret as degrees: msg.x=yaw_deg, msg.y=pitch_deg
        yaw = _clamp(float(msg.x), -self.p.gimbal.clamp_yaw_deg, self.p.gimbal.clamp_yaw_deg)
        pitch = _clamp(float(msg.y), self.p.gimbal.clamp_pitch_min_deg, self.p.gimbal.clamp_pitch_max_deg)

        payload = cmd_gimbal_simple(
            yaw_deg=yaw,
            pitch_deg=pitch,
            spd=self.p.gimbal.default_spd,
            acc=self.p.gimbal.default_acc,
        )
        self._write_serial(payload)

    def _write_serial(self, payload: bytes) -> bool:
        with self._ser_lock:
            ser = self._ser
            if ser is None or not getattr(ser, "is_open", False):
                return False

            try:
                ser.write(payload)
                # flush is optional; usually okay to omit for performance
                self._tx_count += 1
                self._last_tx_mono = _now_mono()
                return True
            except SerialException as e:
                self._serial_errors += 1
                self._warn_throttled(f"Serial TX error: {e}")
                self._close_serial()
                return False
            except Exception as e:
                self._serial_errors += 1
                self._warn_throttled(f"Unexpected TX error: {e}")
                return False

    # ----------------------------------------
    # Health + diagnostics
    # ----------------------------------------
    # Summary: /ugv_base/ping for deterministic health and a diagnostics publisher for observability.

    def _connected_state(self) -> Tuple[bool, int]:
        # Returns (connected, last_rx_age_ms)
        if self._ser is None or not getattr(self._ser, "is_open", False):
            return (False, 10**9)

        if self._last_rx_mono <= 0.0:
            return (False, 10**9)

        age_ms = int((_now_mono() - self._last_rx_mono) * 1000.0)
        ok = age_ms <= self.p.health.max_feedback_age_ms
        return (ok, age_ms)

    def _publish_connected(self) -> None:
        ok, age_ms = self._connected_state()

        if self._last_connected_pub is None or ok != self._last_connected_pub:
            m = Bool()
            m.data = bool(ok)
            self.pub_connected.publish(m)
            self._last_connected_pub = ok

    def _on_ping(self, _req: Trigger.Request, _res: Trigger.Response) -> Trigger.Response:
        ok, age_ms = self._connected_state()
        cmd_age_ms = int((_now_mono() - self._last_cmd_mono) * 1000.0) if self._last_cmd_mono > 0 else 10**9

        resp = Trigger.Response()
        resp.success = bool(ok)

        resp.message = (
            f"connected={ok} last_rx_age_ms={age_ms} "
            f"cmd_vel_age_ms={cmd_age_ms} "
            f"rx={self._rx_count} tx={self._tx_count} "
            f"parse_errors={self._parse_errors} dropped={self._rx_dropped} serial_errors={self._serial_errors}"
        )
        return resp

    def _publish_diagnostics(self) -> None:
        ok, age_ms = self._connected_state()
        cmd_age_ms = int((_now_mono() - self._last_cmd_mono) * 1000.0) if self._last_cmd_mono > 0 else 10**9

        st = DiagnosticStatus()
        st.name = "ugv_base_bridge"
        st.hardware_id = self.p.serial.device
        st.level = DiagnosticStatus.OK if ok else DiagnosticStatus.ERROR
        st.message = "OK" if ok else "NOT_CONNECTED"

        def kv(k: str, v: Any) -> KeyValue:
            item = KeyValue()
            item.key = str(k)
            item.value = str(v)
            return item

        st.values = [
            kv("serial.device", self.p.serial.device),
            kv("serial.baud", self.p.serial.baud),
            kv("connected", ok),
            kv("last_rx_age_ms", age_ms),
            kv("last_tx_age_ms", int((_now_mono() - self._last_tx_mono) * 1000.0) if self._last_tx_mono > 0 else 10**9),
            kv("cmd_vel_age_ms", cmd_age_ms),
            kv("rx_count", self._rx_count),
            kv("tx_count", self._tx_count),
            kv("parse_errors", self._parse_errors),
            kv("rx_dropped", self._rx_dropped),
            kv("serial_errors", self._serial_errors),
            kv("feedback_flow.enabled", self.p.feedback_flow.enabled),
            kv("feedback_flow.interval_ms", self.p.feedback_flow.interval_ms),
            kv("heartbeat.timeout_ms", self.p.heartbeat.timeout_ms),
            kv("uart_echo_mode.enabled", self.p.uart_echo_mode.enabled),
        ]

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self.pub_diag.publish(arr)

    # ----------------------------------------
    # Logging helpers + shutdown
    # ----------------------------------------
    # Summary: avoid log spam and cleanly stop the RX thread.

    def _warn_throttled(self, msg: str) -> None:
        now = _now_mono()
        if (now - self._last_warn_mono) >= self._warn_period_s:
            self.get_logger().warn(msg)
            self._last_warn_mono = now

    def destroy_node(self) -> bool:
        # Stop RX loop
        self._rx_stop.set()
        try:
            if self._rx_thread is not None:
                self._rx_thread.join(timeout=1.0)
        except Exception:
            pass

        self._close_serial()
        return super().destroy_node()


# ----------------------------------------
# Entrypoint
# ----------------------------------------
# Summary: standard rclpy main.

def main() -> None:
    rclpy.init()
    node = UgvBaseBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
