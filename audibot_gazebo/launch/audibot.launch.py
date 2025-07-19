from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    audibot_gazebo_dir = get_package_share_directory('audibot_gazebo')
    audibot_teleop_dir = get_package_share_directory('car_teleop')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(audibot_gazebo_dir, 'launch', 'single_vehicle_example.launch.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(audibot_teleop_dir, 'launch', 'teleop.launch.py')
            )
        ),
        Node(
            package='car_status',
            executable='car_status_node',
            name='car_status_node',
            output='screen'
        )
    ])
