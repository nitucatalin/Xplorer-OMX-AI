#!/usr/bin/env python3
"""
sim.launch.py — pornește Gazebo (Harmonic) cu world-ul de laborator,
spawn-uiește ansamblul Xplorer-A + OMX-AI la HOME (1.0, 0.0) și pornește
robot_state_publisher + ros_gz_bridge.

Modelul implicit este cel REALIST (xplorer_omx_real.urdf, generat din
exportul CAD Onshape cu tools/flatten_onshape_urdf.py). Pentru modelul
geometric simplu: model:=simple.

Utilizare:
  ros2 launch xplorer_omx_sim sim.launch.py
  ros2 launch xplorer_omx_sim sim.launch.py headless:=true   # fără GUI
  ros2 launch xplorer_omx_sim sim.launch.py model:=simple
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('xplorer_omx_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = os.path.join(pkg, 'worlds', 'lab_world.sdf')
    model = 'simple' if 'model:=simple' in ' '.join(sys.argv) else 'real'
    urdf = os.path.join(
        pkg, 'urdf',
        'xplorer_omx.urdf' if model == 'simple' else 'xplorer_omx_real.urdf')
    bridge_cfg = os.path.join(pkg, 'config', 'bridge.yaml')

    with open(urdf, 'r') as f:
        robot_description = f.read()

    # Gazebo rezolva URI-urile package://xplorer_omx_sim/meshes/... daca
    # directorul share al pachetului e in GZ_SIM_RESOURCE_PATH
    resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg))

    headless = LaunchConfiguration('headless')

    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world}'}.items(),
        condition=UnlessCondition(headless),
    )
    gz_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -s {world}'}.items(),
        condition=IfCondition(headless),
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
    )

    # baza modelului real e la inaltimea axelor rotilor (raza 0.0553 m)
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'xplorer_omx',
                   '-x', '1.0', '-y', '0.0',
                   '-z', '0.06' if model == 'real' else '0.14'],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': bridge_cfg,
                     'use_sim_time': True}],
    )

    # reasambleaza ground-truth-ul (robot + obiecte) in /sim/dynamic_poses
    # cu nume corecte (dynamic_pose/info din Harmonic vine fara nume)
    gt_mux = Node(
        package='xplorer_omx_sim',
        executable='ground_truth_mux',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('model', default_value='real',
                              description='real (mesh-uri CAD) sau simple'),
        resource_path,
        gz_gui,
        gz_headless,
        rsp,
        spawn,
        bridge,
        gt_mux,
    ])
