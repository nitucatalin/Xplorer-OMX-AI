#!/usr/bin/env python3
"""
manip.launch.py — pornește nodul de manipulare simulat (echivalentul
start_jetson.sh + manipulation_infer_node de pe Jetson Orin Nano).

Utilizare (după sim.launch.py):
  ros2 launch xplorer_omx_sim manip.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xplorer_omx_sim',
            executable='manipulation_infer_node_sim',
            name='manipulation_infer_node',
            output='screen',
        ),
    ])
