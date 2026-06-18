#!/usr/bin/env python3
"""
nav2.launch.py — stack-ul Nav2 pentru simulare (echivalentul
navxplorer.launch.py de pe RPi5): map_server + AMCL + controller MPPI +
planner + behaviors + bt_navigator, cu lifecycle manager.

AMCL pornește direct cu pose-ul HOME (1.0, 0.0, 0) — set_initial_pose=true
în nav2_params.yaml — deci nu mai e nevoie de 2D Pose Estimate.

Utilizare (după sim.launch.py):
  ros2 launch xplorer_omx_sim nav2.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('xplorer_omx_sim')
    params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    map_yaml = os.path.join(pkg, 'maps', 'lab_map.yaml')

    lifecycle_nodes_loc = ['map_server', 'amcl']
    lifecycle_nodes_nav = ['controller_server', 'planner_server',
                           'behavior_server', 'bt_navigator']

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params, {'yaml_filename': map_yaml}],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True,
                         'autostart': True,
                         'node_names': lifecycle_nodes_loc}],
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': True,
                         'autostart': True,
                         'node_names': lifecycle_nodes_nav}],
        ),
    ])
