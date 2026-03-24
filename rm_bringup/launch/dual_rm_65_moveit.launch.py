"""
MoveIt stack for dual RM65: move_group, robot_state_publisher, optional RViz.

Aligned with ``robot_planning_interface/config/dual_rm65_moveit_config.json`` →
``moveit_config.general_config``:

- ``description_package`` / ``description_file``: ``rm_description`` /
  ``urdf/dual_rm65.urdf`` (included by ``dual_arm_rm_65_config`` xacro)
- ``moveit_config_package`` / ``moveit_config_file``: ``dual_arm_rm_65_config`` /
  ``config/dual_rm65.srdf`` (loaded via MoveItConfigsBuilder)
- ``moveit_rviz_file``: ``config/moveit.rviz``
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

MOVEIT_CONFIG_PACKAGE = "dual_arm_rm_65_config"
MOVEIT_RVIZ_FILE = "config/moveit.rviz"
ROBOT_NAME = "dual_rm65"


def generate_launch_description():
    launch_rviz = LaunchConfiguration("launch_rviz")
    joint_states_topic = LaunchConfiguration("joint_states_topic")
    follow_joint_trajectory_action = LaunchConfiguration("follow_joint_trajectory_action")

    moveit_config = MoveItConfigsBuilder(
        ROBOT_NAME, package_name=MOVEIT_CONFIG_PACKAGE
    ).to_moveit_configs()

    rviz_config_path = os.path.join(
        get_package_share_directory(MOVEIT_CONFIG_PACKAGE),
        MOVEIT_RVIZ_FILE,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="false",
                description="If true, start RViz with the MoveIt config",
            ),
            DeclareLaunchArgument(
                "joint_states_topic",
                default_value="/joint_states",
                description="Remap move_group joint_states subscription",
            ),
            DeclareLaunchArgument(
                "follow_joint_trajectory_action",
                default_value="/dual_arm_controller/follow_joint_trajectory",
                description="Remap trajectory execution action",
            ),
            Node(
                package="rm_bringup",
                executable="standalone_dual_arm_adapter",
                name="standalone_dual_arm_adapter",
                output="screen",
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                remappings=[
                    ("/joint_states", joint_states_topic),
                    ("/follow_joint_trajectory", follow_joint_trajectory_action),
                ],
                parameters=[moveit_config.to_dict()],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="both",
                parameters=[moveit_config.robot_description],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                condition=IfCondition(launch_rviz),
                output="log",
                arguments=["-d", rviz_config_path],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.planning_pipelines,
                    moveit_config.robot_description_kinematics,
                ],
            ),
        ]
    )
