from setuptools import setup

package_name = "ugv_rover_pt_offload"

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
    description="Offload helper (rsync over SSH) for UGV mission bundles.",
    license="MIT",
)
