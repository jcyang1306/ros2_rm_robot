import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    simulation = LaunchConfiguration('simulation')

    # One include: spawns two rm_driver nodes (left_arm / right_arm) from dual_rm_65_driver.launch.py
    dual_rm_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rm_driver'),
                'launch',
                'dual_rm_65_driver.launch.py',
            )
        ),
        launch_arguments=[('simulation', simulation)],
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

    # if launch argument use_planning_adapter is true, spawn the planning adapter node
    if LaunchConfiguration('use_planning_adapter'):
        planning_adapter_node = Node(
            package='rm_bringup',
            executable='standalone_dual_arm_adapter',
            name='standalone_dual_arm_adapter',
            output='screen',
        )
    else:
        planning_adapter_node = None

    return LaunchDescription([
        DeclareLaunchArgument(
            'simulation',
            default_value='false',
            description='If true, use fake_rm_driver (no real arm TCP/UDP).',
        ),
        DeclareLaunchArgument(
            'use_planning_adapter',
            default_value='true',
            description='If true, use the planning adapter node.',
        ),
        dual_rm_driver_launch,
        dual_rm_control_launch,
        planning_adapter_node,
    ])