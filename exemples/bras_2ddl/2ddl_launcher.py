from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='commande',
            namespace='',
            executable='algoTwoDdl',
            name='two_ddl_pos_to_angle'
        ),
        Node(
            package='motors_ctr',
            namespace='',
            executable='motors_ctr',
            name='commande_motors'
        ),
    ])
