# ----------------------------------------
# 1. Bringup Launch
# ----------------------------------------
# Launch camera + gimbal serial node + scan mission node.
# All configuration loaded from params.yaml file.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare params_file argument with default path
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value="/work/params.yaml",
        description="Path to the ROS 2 parameters YAML file"
    )
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        params_file_arg,

        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name="camera",
            parameters=[params_file],
            output="screen",
        ),

        Node(
            package="ugv_rover_pt_gimbal_serial",
            executable="gimbal_node",
            name="gimbal_serial",
            parameters=[params_file],
            output="screen",
        ),

        Node(
            package="ugv_rover_pt_scan_mission",
            executable="scan_mission_node",
            name="scan_mission",
            parameters=[params_file],
            output="screen",
        ),
    ])
