from setuptools import setup

package_name = "ugv_rover_pt_gimbal_serial"

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
    maintainer_email="you@local",
    description="Waveshare UGV Rover PT gimbal serial JSON driver (T=133).",
    license="MIT",
    entry_points={
        "console_scripts": [
            "gimbal_node = ugv_rover_pt_gimbal_serial.gimbal_node:main",
        ],
    },
)
