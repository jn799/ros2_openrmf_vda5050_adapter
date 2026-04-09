from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('vda5050_mock_robot'),
        'config', 'mock_robot.yaml'
    )

    return LaunchDescription([
        Node(
            package='vda5050_mock_robot',
            executable='mock_robot',
            name='vda5050_mock_robot',
            output='screen',
            parameters=[config],
        )
    ])
