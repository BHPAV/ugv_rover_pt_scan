from setuptools import setup

package_name = "ugv_rover_pt_scan_mission"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/action", ["action/ScanAndOffload.action"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="BoxHead",
    maintainer_email="you@local",
    description="Scan mission: sweep gimbal, capture frames, offload, idle.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "scan_mission_node = ugv_rover_pt_scan_mission.scan_mission_node:main",
        ],
    },
)
