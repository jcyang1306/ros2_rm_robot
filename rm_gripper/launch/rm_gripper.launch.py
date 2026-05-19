from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    gripper_type = LaunchConfiguration("gripper_type")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gripper_type",
                default_value="robotiq",
                choices=["robotiq", "hitbot"],
                description="Gripper implementation to run.",
            ),
            Node(
                package="rm_gripper",
                executable="rm_modbus_gripper",
                name="rm_modbus_gripper",
                output="screen",
                arguments=["--gripper-type", gripper_type],
            ),
        ]
    )
