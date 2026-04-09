"""
Full RMF + VDA5050 stack launch file.

Starts (in order of dependency):
  1. rmf_traffic_schedule  — RMF central traffic brain
  2. building_map_server   — publishes /map (BuildingMap) from warehouse.building.yaml
  3. rmf_vda5050_adapter   — our fleet adapter (waits up to 10 s for schedule)
  4. Visualization nodes   — navgraph, fleet states
  5. rviz2                 — pre-configured RMF view

Usage:
  ros2 launch rmf_vda5050_adapter rmf_stack.launch.py

Optional args:
  map_name:=L1            Level to display in RViz (must match building YAML level key)
  headless:=true          Skip rviz2 (useful on headless servers)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('rmf_vda5050_adapter')
    building_map = os.path.join(pkg, 'config', 'warehouse.building.yaml')
    rviz_config = os.path.join(pkg, 'config', 'rmf.rviz')

    map_name = LaunchConfiguration('map_name')
    headless = LaunchConfiguration('headless')

    return LaunchDescription([
        DeclareLaunchArgument('map_name', default_value='L1',
                              description='Level name shown in RViz'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Set true to skip rviz2'),

        # 1. RMF traffic schedule — must start first
        Node(
            package='rmf_traffic_ros2',
            executable='rmf_traffic_schedule',
            name='rmf_traffic_schedule',
            output='screen',
        ),

        # 2. Building map server — publishes /map topic
        Node(
            package='rmf_building_map_tools',
            executable='building_map_server',
            name='building_map_server',
            arguments=[building_map],
            output='screen',
        ),

        # 3a. Task dispatcher — manages task bidding between adapter and schedule
        Node(
            package='rmf_task_ros2',
            executable='rmf_task_dispatcher',
            name='rmf_task_dispatcher',
            output='screen',
        ),

        # 3b. Our adapter — give schedule a moment to come up first
        TimerAction(period=2.0, actions=[
            Node(
                package='rmf_vda5050_adapter',
                executable='adapter',
                name='rmf_vda5050_adapter',
                output='screen',
            ),
        ]),

        # 4a. Nav graph visualizer — renders waypoints and lanes in RViz
        Node(
            package='rmf_visualization_navgraphs',
            executable='navgraph_visualizer_node',
            name='navgraph_visualizer',
            output='screen',
            parameters=[{
                'initial_map_name': map_name,
                'lane_width': 0.5,
                'waypoint_scale': 1.3,
                'text_scale': 0.7,
                'lane_transparency': 0.6,
            }],
        ),

        # 4b. Fleet states visualizer — renders robot positions in RViz
        Node(
            package='rmf_visualization_fleet_states',
            executable='fleetstates_visualizer_node',
            name='fleetstates_visualizer',
            output='screen',
            parameters=[{
                'fleet_state_nose_scale': 0.5,
                # radius param must match fleet name from adapter.yaml
                'vda5050_fleet_radius': 0.3,
            }],
        ),

        # 5. RViz2 — only if not headless
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=UnlessCondition(headless),
        ),
    ])
