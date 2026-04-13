from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('vda5050_nav2_bridge'),
        'config', 'nav2_bridge.yaml'
    )

    return LaunchDescription([
        Node(
            package='vda5050_nav2_bridge',
            executable='nav2_bridge',
            name='vda5050_nav2_bridge',
            output='screen',
            parameters=[config],
        )
    ])
