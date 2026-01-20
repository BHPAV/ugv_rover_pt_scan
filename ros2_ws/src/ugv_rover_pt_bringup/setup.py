from setuptools import setup
import os
from glob import glob

package_name = "ugv_rover_pt_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="BoxHead",
    maintainer_email="dev@example.com",
    description="Bringup launch for UGV Rover PT: camera + gimbal + scan mission",
    license="MIT",
    entry_points={},
)
