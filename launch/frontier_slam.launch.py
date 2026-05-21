"""
Frontier-based exploration.

Prerequisites (must already be running):
  ros2 launch basic_slam step3_octomap.launch.py

This adds:
  - frontier_extractor : /projected_map → /frontier_slam/goal + /frontier_slam/path
  - waypoint_controller: /frontier_slam/path + /StoneFish/Odometry → thruster setpoints

Optional argument:
  depth   Target depth in NED metres (Z-down, so positive = below surface).
          If omitted, the controller locks the robot's depth on first odometry.

Examples:
  ros2 launch frontier_slam frontier_slam.launch.py
  ros2 launch frontier_slam frontier_slam.launch.py depth:=8.0

Visualise in RViz2:
  - MarkerArray  /frontier_slam/frontiers  (cyan = candidates, red = active goal)
  - Image        /frontier_slam/debug_image
  - OccupancyGrid /frontier_slam/inflated_map
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    depth_arg = DeclareLaunchArgument(
        'depth',
        default_value='-1.0',
        description=(
            'Target depth in NED metres (e.g. depth:=8.0). '
            'Omit (or pass depth:=-1) to lock depth automatically from the first odometry reading.'
        ),
    )
    depth = LaunchConfiguration('depth')

    return LaunchDescription([
        depth_arg,
        Node(
            package='frontier_slam',
            executable='frontier_extractor',
            name='frontier_extractor',
            output='screen',
            parameters=[{'depth_setpoint': depth}],
        ),
        Node(
            package='frontier_slam',
            executable='waypoint_controller',
            name='waypoint_controller',
            output='screen',
            parameters=[{'depth_setpoint': depth}],
        ),
    ])
