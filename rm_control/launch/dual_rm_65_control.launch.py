from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():

    # TODO: fix hardcoded namespaces, passing as launch argument
    namespaces = ['left_arm', 'right_arm']
    node_names = ['left_control', 'right_control']
    node_to_launch = [
        Node(
            package='rm_control',
            executable='rm_control',
            name=node_name,          
            namespace=arm_namespace,
            parameters=[
                {'arm_type': 65},
                {'follow': False}
            ],
            output='screen'
        ) for arm_namespace, node_name in zip(namespaces, node_names)

    ]


    return LaunchDescription(node_to_launch)
