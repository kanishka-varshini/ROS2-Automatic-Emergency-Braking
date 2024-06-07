import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='lab1_pkg',
            executable='talker',
            name='talker',
            parameters=[
                {'v': 1.0},  # Initial speed
                {'d': 0.0},  # Initial steering angle
            ]
        ),
        Node(
            package='lab1_pkg',
            executable='relay',
            name='relay'
        ),
        Node(
            package='lab1_pkg',
            executable='safety',
            name='safety'
        )
    ])