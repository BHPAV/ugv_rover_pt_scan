# ----------------------------------------
# 1. Gimbal Node
# ----------------------------------------
# Subscribes to /gimbal/setpoint (geometry_msgs/Vector3: x=yaw_deg, y=pitch_deg)
# Sends JSON {"T":133,"X":yaw,"Y":pitch,"SPD":spd,"ACC":acc} over UART.

import json
import os
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
        super().__init__("ugv_rover_pt_gimbal_serial")

        port = self.declare_parameter("port", os.getenv("UGV_SERIAL_PORT", "/dev/ttyTHS1")).value
        baud = int(self.declare_parameter("baud", int(os.getenv("UGV_SERIAL_BAUD", "115200"))).value)
        spd = int(self.declare_parameter("spd", 0).value)
        acc = int(self.declare_parameter("acc", 0).value)

        self.params = GimbalParams(port=str(port), baud=int(baud), spd=int(spd), acc=int(acc))
        self.get_logger().info(f"Opening serial {self.params.port} @ {self.params.baud}")

        self.ser = serial.Serial(self.params.port, baudrate=self.params.baud, timeout=0.2)
        self.sub = self.create_subscription(Vector3, "/gimbal/setpoint", self.on_setpoint, 10)

        # Center on start
        self.send_pose(0.0, 0.0)

    def send_pose(self, yaw_deg: float, pitch_deg: float) -> None:
        payload = {
            "T": 133,
            "X": clamp(float(yaw_deg), -180.0, 180.0),
            "Y": clamp(float(pitch_deg), -30.0, 90.0),
            "SPD": int(self.params.spd),
            "ACC": int(self.params.acc),
        }
        msg = json.dumps(payload).encode("utf-8")
        self.ser.write(msg)

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
