import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rc_car_driver'),
        'config', 'vehicle_driver.yaml')

    return LaunchDescription([
        Node(
            package='rc_car_driver',
            executable='vehicle_driver',
            name='vehicle_driver',
            output='screen',
            parameters=[config],
        ),
    ])
