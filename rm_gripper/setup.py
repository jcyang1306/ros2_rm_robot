from glob import glob
from setuptools import setup

package_name = "rm_gripper"

setup(
    name=package_name,
    version="0.0.0",
    py_modules=[
        "rm_modbus_gripper",
        "rm_485",
    ],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hkclr",
    maintainer_email="jcyang@hkclr.hk",
    description="ROS 2 Python package for RM Modbus gripper control.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rm_modbus_gripper = rm_modbus_gripper:main",
        ],
    },
)
