"""
MoveIt stack for dual RM65: move_group, robot_state_publisher, optional RViz.

The  gripper  launch argument is forwarded to both the URDF xacro
(dual_rm65.urdf.xacro) and the SRDF xacro (dual_rm65.srdf.xacro) so that
end-effector geometry and its allowed-collision pairs are always in sync.

Supported gripper values
  robotiq_2f85  (default)  Robotiq 2F-85 static gripper
  none                     Bare flange – no gripper

MoveItConfigsBuilder must run inside OpaqueFunction because it reads and
processes xacro files immediately; OpaqueFunction defers execution until
after LaunchConfiguration values have been resolved.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

MOVEIT_CONFIG_PACKAGE = "dual_arm_rm_65_config"
MOVEIT_RVIZ_FILE = "config/moveit.rviz"
ROBOT_NAME = "dual_rm65"


def _create_nodes(context, *args, **kwargs):
    """Deferred setup: called at launch time after all args are resolved."""
    gripper = LaunchConfiguration("gripper").perform(context)

    moveit_config = (
        MoveItConfigsBuilder(ROBOT_NAME, package_name=MOVEIT_CONFIG_PACKAGE)
        # Pass gripper arg to the URDF xacro → controls which end-effector
        # geometry is loaded.
        .robot_description(
            file_path="config/dual_rm65.urdf.xacro",
            mappings={"gripper": gripper},
        )
        # Pass gripper arg to the SRDF xacro → controls which
        # disable_collisions entries are active.
        .robot_description_semantic(
            file_path="config/dual_rm65.srdf.xacro",
            mappings={"gripper": gripper},
        )
        .to_moveit_configs()
    )

    rviz_config_path = os.path.join(
        get_package_share_directory(MOVEIT_CONFIG_PACKAGE),
        MOVEIT_RVIZ_FILE,
    )

    joint_states_topic = LaunchConfiguration("joint_states_topic")
    follow_joint_trajectory_action = LaunchConfiguration("follow_joint_trajectory_action")
    launch_rviz = LaunchConfiguration("launch_rviz")

    return [
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


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gripper",
                default_value="robotiq_2f85",
                description=(
                    "End-effector type attached to both arms. "
                    "Supported: 'robotiq_2f85', 'none'."
                ),
            ),
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
            # OpaqueFunction defers MoveItConfigsBuilder construction until
            # after all LaunchConfiguration values (e.g. gripper) are resolved.
            OpaqueFunction(function=_create_nodes),
        ]
    )
