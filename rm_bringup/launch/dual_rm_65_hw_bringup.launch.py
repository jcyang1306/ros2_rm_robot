import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # One include: spawns two rm_driver nodes (left_arm / right_arm) from dual_rm_65_driver.launch.py
    dual_rm_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rm_driver'),
                'launch',
                'dual_rm_65_driver.launch.py',
            )
        )
    )

    dual_rm_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rm_control'),
                'launch',
                'dual_rm_65_control.launch.py',
            )
        )
    )

    return LaunchDescription([dual_rm_driver_launch, dual_rm_control_launch])