# ----------------------------------------
# 1. Scan mission action server
# ----------------------------------------
# Sweeps gimbal via /gimbal/setpoint, captures /image_raw, saves jpeg frames,
# offloads to NAS via rsync, returns to idle.

import json
import time
from pathlib import Path
from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge

from ugv_rover_pt_interfaces.action import ScanAndOffload
from ugv_rover_pt_offload.offload import OffloadTarget, offload_with_markers

import cv2


class ScanMission(Node):
    def __init__(self) -> None:
        super().__init__("scan_mission")

        self.bridge = CvBridge()
        self.latest: Optional[Image] = None

        # All config comes from params.yaml via launch file
        self.mission_root = Path(str(self.declare_parameter("mission_root", "/data/missions").value))
        self.nas_host = str(self.declare_parameter("nas_host", "192.168.1.20").value)
        self.nas_user = str(self.declare_parameter("nas_user", "boxhead").value)
        self.nas_dest = str(self.declare_parameter("nas_dest_dir", "/share/boxhead/ugv_rover_pt/missions/").value)

        self.sub = self.create_subscription(Image, "/image_raw", self.on_image, 10)
        self.pub = self.create_publisher(Vector3, "/gimbal/setpoint", 10)

        self.server = ActionServer(self, ScanAndOffload, "scan_and_offload", self.execute)

        # Center gimbal on start
        self.set_gimbal(0.0, 0.0)

    def on_image(self, msg: Image) -> None:
        self.latest = msg

    def set_gimbal(self, yaw_deg: float, pitch_deg: float) -> None:
        cmd = Vector3()
        cmd.x = float(yaw_deg)
        cmd.y = float(pitch_deg)
        cmd.z = 0.0
        self.pub.publish(cmd)

    def wait_for_frame(self, timeout_sec: float = 5.0) -> bool:
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < timeout_sec:
            if self.latest is not None:
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return False

    def save_latest_jpeg(self, out_path: Path) -> None:
        assert self.latest is not None
        img = self.bridge.imgmsg_to_cv2(self.latest, desired_encoding="bgr8")
        out_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(out_path), img)

    async def execute(self, goal_handle):
        g = goal_handle.request

        duration_sec = float(g.duration_sec) if g.duration_sec > 0 else 30.0
        yaw_min = float(g.yaw_min_deg) if g.yaw_min_deg != 0 else -160.0
        yaw_max = float(g.yaw_max_deg) if g.yaw_max_deg != 0 else 160.0
        yaw_step = float(g.yaw_step_deg) if g.yaw_step_deg != 0 else 15.0
        pitch_list: List[float] = list(g.pitch_list_deg) if len(g.pitch_list_deg) > 0 else [-20.0, 0.0, 20.0]
        frames_per_pose = int(g.frames_per_pose) if g.frames_per_pose > 0 else 3
        settle_ms = int(g.settle_ms) if g.settle_ms > 0 else 200

        ts = time.strftime("%Y-%m-%dT%H-%M-%S")
        bundle = self.mission_root / f"mission_{ts}"
        frames_dir = bundle / "frames"
        bundle.mkdir(parents=True, exist_ok=True)

        fb = ScanAndOffload.Feedback()
        fb.state = "WAIT_IMAGE"
        fb.progress = 0.0
        goal_handle.publish_feedback(fb)

        if not self.wait_for_frame():
            res = ScanAndOffload.Result()
            res.success = False
            res.local_bundle = str(bundle)
            res.nas_bundle = ""
            res.message = "No frames on /image_raw"
            goal_handle.abort()
            return res

        fb.state = "SCAN"
        goal_handle.publish_feedback(fb)

        start = time.time()
        total_poses = max(1, int((yaw_max - yaw_min) / max(1e-6, yaw_step)) + 1) * max(1, len(pitch_list))
        pose_idx = 0
        img_idx = 0

        yaw = yaw_min
        while rclpy.ok() and yaw <= yaw_max and (time.time() - start) < duration_sec:
            for pitch in pitch_list:
                self.set_gimbal(yaw, pitch)
                time.sleep(settle_ms / 1000.0)

                for k in range(frames_per_pose):
                    if self.latest is None:
                        continue
                    fname = f"yaw_{yaw:+06.1f}_pitch_{pitch:+05.1f}_i_{img_idx:06d}_k_{k}.jpg"
                    self.save_latest_jpeg(frames_dir / fname)
                    img_idx += 1
                    time.sleep(0.05)

                pose_idx += 1
                fb.progress = min(0.95, pose_idx / float(total_poses))
                goal_handle.publish_feedback(fb)

            yaw += yaw_step

        self.set_gimbal(0.0, 0.0)

        fb.state = "PACKAGE"
        fb.progress = 0.96
        goal_handle.publish_feedback(fb)

        meta = {
            "timestamp": ts,
            "duration_sec": duration_sec,
            "yaw": {"min": yaw_min, "max": yaw_max, "step": yaw_step},
            "pitch_list": pitch_list,
            "frames_per_pose": frames_per_pose,
            "settle_ms": settle_ms,
            "frames_saved": img_idx,
            "nas": {"host": self.nas_host, "user": self.nas_user, "dest_dir": self.nas_dest},
        }
        (bundle / "meta.json").write_text(json.dumps(meta, indent=2))

        fb.state = "OFFLOAD"
        fb.progress = 0.98
        goal_handle.publish_feedback(fb)

        try:
            target = OffloadTarget(host=str(self.nas_host), user=str(self.nas_user), dest_dir=str(self.nas_dest))
            nas_path = offload_with_markers(bundle, target)
        except Exception as e:
            res = ScanAndOffload.Result()
            res.success = False
            res.local_bundle = str(bundle)
            res.nas_bundle = ""
            res.message = f"Offload failed: {e}"
            goal_handle.abort()
            return res

        fb.state = "IDLE"
        fb.progress = 1.0
        goal_handle.publish_feedback(fb)

        res = ScanAndOffload.Result()
        res.success = True
        res.local_bundle = str(bundle)
        res.nas_bundle = nas_path
        res.message = "Scan + offload complete"
        goal_handle.succeed()
        return res


def main() -> None:
    rclpy.init()
    node = ScanMission()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
