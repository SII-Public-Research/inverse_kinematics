from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='commande',
            namespace='',
            executable='algo_4dof',
            name='4dof_pos_to_angle'
        ),
        Node(
            package='motors_ctr',
            namespace='',
            executable='motors_4dof',
            name='motors_command'
        ),
    ])
