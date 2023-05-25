from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='command',
            namespace='',
            executable='algoThreeDof',
            name='three_dof_pos_to_angle'
        ),
        Node(
            package='motors_ctr',
            namespace='',
            executable='motors_ctr',
            name='three_dof_motors_command'
        ),
    ])
