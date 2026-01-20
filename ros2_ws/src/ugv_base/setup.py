# ----------------------------------------
# setup.py
# ----------------------------------------
# Summary: ament_python setup for ugv_base with console_script entrypoint.

from setuptools import setup

package_name = "ugv_base"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="BoxHead",
    maintainer_email="dev@example.com",
    description="ROS 2 bridge for Waveshare UGV Rover ESP32 UART JSON protocol.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "bridge_node = ugv_base.bridge_node:main",
        ],
    },
)
