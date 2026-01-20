#!/usr/bin/env python3
"""
UGV Rover PT Status TUI - Real-time device monitoring

Run inside the Docker container:
    python3 /work/scripts/status_tui.py

Or from host:
    ./scripts/run_status_tui.sh
"""

import json
import sys
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from std_msgs.msg import String, Bool
    from std_srvs.srv import Trigger
    from sensor_msgs.msg import Image
except ImportError:
    print("Error: ROS 2 not available. Run this inside the container:")
    print("  docker compose exec ugv_ros python3 /work/scripts/status_tui.py")
    sys.exit(1)

try:
    from rich.console import Console
    from rich.layout import Layout
    from rich.live import Live
    from rich.panel import Panel
    from rich.table import Table
    from rich.text import Text
    RICH_AVAILABLE = True
except ImportError:
    RICH_AVAILABLE = False


@dataclass
class DeviceStatus:
    """Container for all device status information."""
    # Connection
    connected: bool = False
    last_ping: Optional[datetime] = None
    ping_stats: str = "No data"

    # ESP32 feedback
    last_feedback: Optional[datetime] = None
    feedback_json: dict = field(default_factory=dict)
    feedback_rate_hz: float = 0.0
    feedback_count: int = 0

    # Camera
    camera_connected: bool = False
    camera_rate_hz: float = 0.0
    last_image: Optional[datetime] = None
    image_count: int = 0
    image_size: str = "N/A"

    # Gimbal (from feedback)
    gimbal_pan: float = 0.0
    gimbal_tilt: float = 0.0

    # Motors (from feedback)
    motor_left: float = 0.0
    motor_right: float = 0.0

    # IMU (from feedback)
    accel: tuple = (0.0, 0.0, 0.0)
    gyro: tuple = (0.0, 0.0, 0.0)

    # Battery
    voltage: float = 0.0

    # Odometry
    odom_left: int = 0
    odom_right: int = 0


class StatusMonitor(Node):
    """ROS 2 node that monitors UGV device status."""

    def __init__(self, status: DeviceStatus):
        super().__init__('status_tui')
        self.status = status
        self._feedback_times = []
        self._image_times = []

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribe to feedback
        self.create_subscription(
            String,
            '/ugv_base/feedback/json',
            self._feedback_callback,
            10
        )

        self.create_subscription(
            Bool,
            '/ugv_base/feedback/connected',
            self._connected_callback,
            10
        )

        # Subscribe to camera (best effort for high-rate sensor data)
        self.create_subscription(
            Image,
            '/image_raw',
            self._image_callback,
            sensor_qos
        )

        # Create ping service client
        self.ping_client = self.create_client(Trigger, '/ugv_base/ping')

        # Timer for periodic ping
        self.create_timer(2.0, self._ping_callback)

        # Timer to calculate rates
        self.create_timer(1.0, self._calculate_rates)

    def _feedback_callback(self, msg: String):
        """Handle ESP32 feedback JSON."""
        self.status.last_feedback = datetime.now()
        self.status.feedback_count += 1
        self._feedback_times.append(time.time())

        try:
            data = json.loads(msg.data)
            self.status.feedback_json = data

            # Extract T=1001 sensor feedback
            if data.get('T') == 1001:
                self.status.motor_left = data.get('L', 0.0)
                self.status.motor_right = data.get('R', 0.0)
                self.status.gimbal_pan = data.get('pan', 0.0)
                self.status.gimbal_tilt = data.get('tilt', 0.0)
                # Voltage is in centivolts (12.03V = 1203) from ESP32
                self.status.voltage = data.get('v', 0.0) / 100.0
                self.status.odom_left = data.get('odl', 0)
                self.status.odom_right = data.get('odr', 0)
                self.status.accel = (
                    data.get('ax', 0.0),
                    data.get('ay', 0.0),
                    data.get('az', 0.0)
                )
                self.status.gyro = (
                    data.get('gx', 0.0),
                    data.get('gy', 0.0),
                    data.get('gz', 0.0)
                )
        except json.JSONDecodeError:
            pass

    def _connected_callback(self, msg: Bool):
        """Handle connection status."""
        self.status.connected = msg.data

    def _image_callback(self, msg: Image):
        """Handle camera images."""
        self.status.last_image = datetime.now()
        self.status.image_count += 1
        self.status.camera_connected = True
        self.status.image_size = f"{msg.width}x{msg.height}"
        self._image_times.append(time.time())

    def _ping_callback(self):
        """Periodic ping to ugv_base."""
        if not self.ping_client.wait_for_service(timeout_sec=0.5):
            self.status.ping_stats = "Service unavailable"
            return

        future = self.ping_client.call_async(Trigger.Request())
        future.add_done_callback(self._ping_done)

    def _ping_done(self, future):
        """Handle ping response."""
        try:
            result = future.result()
            self.status.last_ping = datetime.now()
            if result.success:
                self.status.connected = True
                self.status.ping_stats = result.message
            else:
                self.status.ping_stats = f"Failed: {result.message}"
        except Exception as e:
            self.status.ping_stats = f"Error: {e}"

    def _calculate_rates(self):
        """Calculate message rates."""
        now = time.time()
        window = 5.0  # 5 second window

        # Feedback rate
        self._feedback_times = [t for t in self._feedback_times if now - t < window]
        if len(self._feedback_times) >= 2:
            self.status.feedback_rate_hz = len(self._feedback_times) / window

        # Image rate
        self._image_times = [t for t in self._image_times if now - t < window]
        if len(self._image_times) >= 2:
            self.status.camera_rate_hz = len(self._image_times) / window

        # Check camera timeout
        if self.status.last_image:
            age = (datetime.now() - self.status.last_image).total_seconds()
            if age > 2.0:
                self.status.camera_connected = False


def format_age(dt: Optional[datetime]) -> str:
    """Format datetime as age string."""
    if dt is None:
        return "Never"
    age = (datetime.now() - dt).total_seconds()
    if age < 1:
        return "Just now"
    elif age < 60:
        return f"{age:.0f}s ago"
    else:
        return f"{age/60:.1f}m ago"


def create_rich_display(status: DeviceStatus) -> Layout:
    """Create rich layout for the TUI."""
    layout = Layout()

    layout.split_column(
        Layout(name="header", size=3),
        Layout(name="body"),
        Layout(name="footer", size=3),
    )

    layout["body"].split_row(
        Layout(name="left"),
        Layout(name="right"),
    )

    # Header
    header = Table.grid(expand=True)
    header.add_column(justify="center")
    header.add_row(Text("UGV Rover PT Status Monitor", style="bold magenta"))
    layout["header"].update(Panel(header, style="blue"))

    # Connection Panel
    conn_table = Table(show_header=False, box=None, padding=(0, 1))
    conn_table.add_column("Key", style="cyan")
    conn_table.add_column("Value")

    conn_status = "[green]Connected[/]" if status.connected else "[red]Disconnected[/]"
    conn_table.add_row("ESP32:", conn_status)
    conn_table.add_row("Last Ping:", format_age(status.last_ping))
    conn_table.add_row("Stats:", status.ping_stats[:50] if status.ping_stats else "N/A")

    cam_status = "[green]Active[/]" if status.camera_connected else "[red]Inactive[/]"
    conn_table.add_row("Camera:", cam_status)
    conn_table.add_row("Resolution:", status.image_size)
    conn_table.add_row("Frame Rate:", f"{status.camera_rate_hz:.1f} Hz")

    # Sensors Panel
    sensor_table = Table(show_header=False, box=None, padding=(0, 1))
    sensor_table.add_column("Key", style="cyan")
    sensor_table.add_column("Value")

    sensor_table.add_row("Feedback Rate:", f"{status.feedback_rate_hz:.1f} Hz")
    sensor_table.add_row("Last Feedback:", format_age(status.last_feedback))
    sensor_table.add_row("", "")
    sensor_table.add_row("Gimbal Pan:", f"{status.gimbal_pan:.1f}deg")
    sensor_table.add_row("Gimbal Tilt:", f"{status.gimbal_tilt:.1f}deg")
    sensor_table.add_row("", "")
    sensor_table.add_row("Motor L/R:", f"{status.motor_left:.2f} / {status.motor_right:.2f}")
    sensor_table.add_row("Odom L/R:", f"{status.odom_left} / {status.odom_right}")

    # IMU Panel
    imu_table = Table(show_header=False, box=None, padding=(0, 1))
    imu_table.add_column("Key", style="cyan")
    imu_table.add_column("Value")

    imu_table.add_row("Accel X:", f"{status.accel[0]:.3f}")
    imu_table.add_row("Accel Y:", f"{status.accel[1]:.3f}")
    imu_table.add_row("Accel Z:", f"{status.accel[2]:.3f}")
    imu_table.add_row("", "")
    imu_table.add_row("Gyro X:", f"{status.gyro[0]:.3f}")
    imu_table.add_row("Gyro Y:", f"{status.gyro[1]:.3f}")
    imu_table.add_row("Gyro Z:", f"{status.gyro[2]:.3f}")

    # Battery Panel
    battery_table = Table(show_header=False, box=None, padding=(0, 1))
    battery_table.add_column("Key", style="cyan")
    battery_table.add_column("Value")

    voltage = status.voltage
    if voltage > 11.5:
        volt_style = "green"
    elif voltage > 10.5:
        volt_style = "yellow"
    else:
        volt_style = "red"

    battery_table.add_row("Voltage:", f"[{volt_style}]{voltage:.2f}V[/]")

    # 3S LiPo: 12.6V full, 11.1V nominal, 10.0V low
    if voltage > 0:
        pct = min(100, max(0, (voltage - 10.0) / (12.6 - 10.0) * 100))
        battery_table.add_row("Estimated:", f"[{volt_style}]{pct:.0f}%[/]")

    # Combine panels
    layout["left"].split_column(
        Layout(Panel(conn_table, title="Connection", border_style="green")),
        Layout(Panel(battery_table, title="Battery", border_style="yellow")),
    )

    layout["right"].split_column(
        Layout(Panel(sensor_table, title="Sensors", border_style="blue")),
        Layout(Panel(imu_table, title="IMU", border_style="cyan")),
    )

    # Footer
    footer = Table.grid(expand=True)
    footer.add_column(justify="center")
    footer.add_row(Text("Press Ctrl+C to exit", style="dim"))
    layout["footer"].update(Panel(footer, style="dim"))

    return layout


def run_simple_display(status: DeviceStatus):
    """Simple terminal display without rich."""
    import os

    while True:
        # Clear screen
        os.system('clear' if os.name != 'nt' else 'cls')

        print("=" * 50)
        print("     UGV Rover PT Status Monitor")
        print("=" * 50)
        print()

        conn = "Connected" if status.connected else "Disconnected"
        print(f"ESP32:        {conn}")
        print(f"Last Ping:    {format_age(status.last_ping)}")
        print(f"Stats:        {status.ping_stats[:40]}")
        print()

        cam = "Active" if status.camera_connected else "Inactive"
        print(f"Camera:       {cam}")
        print(f"Resolution:   {status.image_size}")
        print(f"Frame Rate:   {status.camera_rate_hz:.1f} Hz")
        print()

        print(f"Feedback:     {status.feedback_rate_hz:.1f} Hz")
        print(f"Gimbal:       pan={status.gimbal_pan:.1f} tilt={status.gimbal_tilt:.1f}")
        print(f"Motors L/R:   {status.motor_left:.2f} / {status.motor_right:.2f}")
        print(f"Odom L/R:     {status.odom_left} / {status.odom_right}")
        print()

        print(f"Voltage:      {status.voltage:.2f}V")
        print()

        print("-" * 50)
        print("Press Ctrl+C to exit")

        time.sleep(0.5)


def main():
    rclpy.init()

    status = DeviceStatus()
    node = StatusMonitor(status)

    # Spin in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    console = Console() if RICH_AVAILABLE else None

    try:
        if RICH_AVAILABLE:
            with Live(create_rich_display(status), console=console, refresh_per_second=4) as live:
                while rclpy.ok():
                    live.update(create_rich_display(status))
                    time.sleep(0.25)
        else:
            print("Note: Install 'rich' for a better TUI: pip install rich")
            run_simple_display(status)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
