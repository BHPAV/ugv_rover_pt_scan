# ----------------------------------------
# 1. Bringup Launch
# ----------------------------------------
# Launch camera + gimbal serial node + scan mission node.

import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = os.getenv("UGV_SERIAL_PORT", "/dev/ttyTHS1")
    serial_baud = int(os.getenv("UGV_SERIAL_BAUD", "115200"))

    cam_dev = os.getenv("UGV_CAMERA_DEVICE", "/dev/video0")
    cam_fps = int(os.getenv("UGV_CAMERA_FPS", "30"))
    cam_w = int(os.getenv("UGV_CAMERA_WIDTH", "1280"))
    cam_h = int(os.getenv("UGV_CAMERA_HEIGHT", "720"))

    mission_root = os.getenv("UGV_MISSION_ROOT", "/data/missions")

    nas_host = os.getenv("NAS_HOST", "box-nas")
    nas_user = os.getenv("NAS_USER", "rover_ingest")
    nas_dest = os.getenv("NAS_DEST_DIR", "/share/rover_ingest/ugv_rover_pt/box-rover/")

    return LaunchDescription([
        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name="camera",
            parameters=[{
                "video_device": cam_dev,
                "image_size": [cam_w, cam_h],
                "time_per_frame": [1, cam_fps],
            }],
            output="screen",
        ),
        Node(
            package="ugv_rover_pt_gimbal_serial",
            executable="gimbal_node",
            name="gimbal_serial",
            parameters=[{
                "port": serial_port,
                "baud": serial_baud,
                "spd": 0,
                "acc": 0,
            }],
            output="screen",
        ),
        Node(
            package="ugv_rover_pt_scan_mission",
            executable="scan_mission_node",
            name="scan_mission",
            parameters=[{
                "mission_root": mission_root,
                "nas_host": nas_host,
                "nas_user": nas_user,
                "nas_dest_dir": nas_dest,
            }],
            output="screen",
        ),
    ])
