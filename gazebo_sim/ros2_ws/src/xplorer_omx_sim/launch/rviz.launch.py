#!/usr/bin/env python3
"""
rviz.launch.py — RViz cu configuratia proprie a pachetului (doar pluginuri
standard rviz_default_plugins, deci fara erori "failed to load plugins").

Utilizare:
  ros2 launch xplorer_omx_sim rviz.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('xplorer_omx_sim')
    cfg = os.path.join(pkg, 'config', 'sim.rviz')
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', cfg],
            parameters=[{'use_sim_time': True}],
        ),
    ])
