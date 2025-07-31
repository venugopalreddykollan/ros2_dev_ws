#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the path to the config files
    trajectory_config_file = os.path.join(
        get_package_share_directory('ros2_traj_pkg'),
        'config',
        'trajectory_params.yaml'
    )
    
    velocity_filter_config_file = os.path.join(
        get_package_share_directory('ros2_traj_pkg'),
        'config',
        'velocity_filter_params.yaml'
    )

    return LaunchDescription([
        # Launch trajectory planner node
        Node(
            package='ros2_traj_pkg',
            executable='trajectory_planner_node',
            name='trajectory_planner',
            parameters=[trajectory_config_file],
            output='screen',
            emulate_tty=True
        ),
        
        # Launch velocity filter node
        Node(
            package='ros2_traj_pkg',
            executable='velocity_filter_node',
            name='velocity_filter',
            parameters=[velocity_filter_config_file],
            output='screen',
            emulate_tty=True
        )
    ])
