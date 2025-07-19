from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.05,
                'autorepeat_rate': 20.0
            }]
        ),
        Node(
            package='car_teleop',
            executable='teleop',
            name='teleop',
            output='screen'
        )
    ])
