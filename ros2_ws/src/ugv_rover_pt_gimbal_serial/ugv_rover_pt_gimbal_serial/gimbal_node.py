# ----------------------------------------
# 1. Gimbal Node
# ----------------------------------------
# Subscribes to /gimbal/setpoint (geometry_msgs/Vector3: x=yaw_deg, y=pitch_deg)
# Sends JSON {"T":133,"X":yaw,"Y":pitch,"SPD":spd,"ACC":acc} over UART.
#
# IMPORTANT: Waveshare ESP32 requires initialization sequence before gimbal
# commands are accepted. Must send T=4 cmd=2 to select gimbal module type.

import json
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

import serial


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


@dataclass(frozen=True)
class GimbalParams:
    port: str
    baud: int
    spd: int
    acc: int


class GimbalNode(Node):
    def __init__(self) -> None:
        super().__init__("gimbal_serial")

        # All config comes from params.yaml via launch file
        port = str(self.declare_parameter("port", "/dev/ttyTHS1").value)
        baud = int(self.declare_parameter("baud", 115200).value)
        spd = int(self.declare_parameter("spd", 200).value)
        acc = int(self.declare_parameter("acc", 10).value)

        self.params = GimbalParams(port=port, baud=baud, spd=spd, acc=acc)
        self.get_logger().info(f"Opening serial {self.params.port} @ {self.params.baud}")

        self.ser = serial.Serial(self.params.port, baudrate=self.params.baud, timeout=0.2)
        self.sub = self.create_subscription(Vector3, "/gimbal/setpoint", self.on_setpoint, 10)

        # Initialize ESP32 for gimbal mode (required before gimbal commands work)
        self._initialize_gimbal()

        # Center on start
        self.send_pose(0.0, 0.0)

    def _send_json(self, payload: dict) -> None:
        """Send JSON command with newline terminator (Waveshare protocol)."""
        msg = (json.dumps(payload) + '\n').encode("utf-8")
        self.ser.write(msg)
        self.ser.flush()

    def _initialize_gimbal(self) -> None:
        """Send initialization sequence to enable gimbal mode on ESP32."""
        self.get_logger().info("Initializing ESP32 gimbal mode...")

        # Set feedback interval
        self._send_json({"T": 142, "cmd": 50})
        time.sleep(0.1)

        # Enable serial feedback flow
        self._send_json({"T": 131, "cmd": 1})
        time.sleep(0.1)

        # Disable serial echo
        self._send_json({"T": 143, "cmd": 0})
        time.sleep(0.1)

        # SELECT MODULE TYPE: 0=None, 1=RoArm-M2-S, 2=Gimbal
        self._send_json({"T": 4, "cmd": 2})
        time.sleep(0.5)

        self.get_logger().info("ESP32 gimbal initialization complete")

    def send_pose(self, yaw_deg: float, pitch_deg: float) -> None:
        payload = {
            "T": 133,
            "X": clamp(float(yaw_deg), -180.0, 180.0),
            "Y": clamp(float(pitch_deg), -30.0, 90.0),
            "SPD": int(self.params.spd),
            "ACC": int(self.params.acc),
        }
        self._send_json(payload)

    def on_setpoint(self, msg: Vector3) -> None:
        self.send_pose(msg.x, msg.y)

    def destroy_node(self) -> bool:
        try:
            self.send_pose(0.0, 0.0)
            time.sleep(0.1)
            self.ser.close()
        except Exception:
            pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = GimbalNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
