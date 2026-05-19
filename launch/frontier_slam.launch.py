"""
Frontier-based exploration.

Prerequisites (must already be running):
  ros2 launch basic_slam step3_octomap.launch.py

This adds:
  - frontier_extractor : /projected_map → /frontier_slam/goal + /frontier_slam/frontiers
  - waypoint_controller: /frontier_slam/goal + /StoneFish/Odometry → thruster setpoints

Visualise in RViz2:
  - MarkerArray  /frontier_slam/frontiers  (cyan = candidates, red = active goal)
  - PointStamped /frontier_slam/goal
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='frontier_slam',
            executable='frontier_extractor',
            name='frontier_extractor',
            output='screen',
        ),
        Node(
            package='frontier_slam',
            executable='waypoint_controller',
            name='waypoint_controller',
            output='screen',
        ),
    ])
