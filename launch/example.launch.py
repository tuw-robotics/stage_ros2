#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution 
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Add string commands if not empty

def generate_launch_description():


    use_sim_time     = LaunchConfiguration('use_sim_time',  default='true')
    this_directory = get_package_share_directory('stage_ros2')
    launch_dir = os.path.join(this_directory, 'launch')
    stage = LaunchConfiguration('stage')
    rviz = LaunchConfiguration('rviz')
    config = LaunchConfiguration('config')
    world = LaunchConfiguration('world')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_stage_cmd = DeclareLaunchArgument(
        'stage',
        default_value='True',
        description='Whether run a stage')

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Whether run a rviz')

    declare_config = DeclareLaunchArgument(
        'config', default_value='example',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_world = DeclareLaunchArgument(
        'world', default_value='example',
        description='world to load in stage')

    return LaunchDescription([
        declare_namespace_cmd,
        declare_rviz_cmd,
        declare_stage_cmd,
        declare_config,
        declare_world,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz.launch.py')),
            condition=IfCondition(rviz),
            launch_arguments={'namespace': namespace,
                              'config': config}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'stage.launch.py')),
            condition=IfCondition(stage),
            launch_arguments={'world': world}.items()),

    ])