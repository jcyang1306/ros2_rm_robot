import launch
import os
import yaml
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    arm_configs = [
        os.path.join(get_package_share_directory('rm_driver'), 'config', 'rm_65_left_arm_config.yaml'),
        os.path.join(get_package_share_directory('rm_driver'), 'config', 'rm_65_right_arm_config.yaml'),
    ]
    node_names = ['left_rm_driver', 'right_rm_driver']

    node_to_launch = []

    for arm_config, node_name in zip(arm_configs, node_names):
        with open(arm_config,'r') as f:
            arm_raw = yaml.safe_load(f)
            arm_params = arm_raw.get('rm_driver', {}).get('ros__parameters', {})
            node_to_launch.append(Node(
                name= node_name,
                package= "rm_driver",
                executable= "rm_driver",
                namespace= arm_params.get('namespace', ''),
                parameters= [arm_params],
                output= 'screen'
            ))
    return LaunchDescription(node_to_launch)