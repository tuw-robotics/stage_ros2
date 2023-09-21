#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time',  default='true')
    this_directory = get_package_share_directory('stage_ros2')
    launch_dir = os.path.join(this_directory, 'launch')

    enforce_prefixes = LaunchConfiguration('enforce_prefixes')
    enforce_prefixes_cmd = DeclareLaunchArgument(
        'enforce_prefixes',
        default_value='false',
        description='on true a prefixes are used for a single robot environment')
    

    one_tf_tree = LaunchConfiguration('one_tf_tree')
    one_tf_tree_cmd = DeclareLaunchArgument(
        'one_tf_tree',
        default_value='false',
        description='on true all tfs are published with a namespace on /tf and /tf_static')
   
    namespace = LaunchConfiguration('namespace')
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    stage = LaunchConfiguration('stage')
    declare_stage_cmd = DeclareLaunchArgument(
        'stage',
        default_value='True',
        description='Whether run a stage')

    rviz = LaunchConfiguration('rviz')
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Whether run a rviz')

    world = LaunchConfiguration('world')
    declare_world = DeclareLaunchArgument(
        'world', default_value='cave_three_robots',
        description='world to load in stage and rviz config [cave, example]')

    return LaunchDescription([
        declare_namespace_cmd,
        declare_rviz_cmd,
        declare_stage_cmd,
        enforce_prefixes_cmd,
        one_tf_tree_cmd,
        declare_world,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz.launch.py')),
            condition=IfCondition(rviz),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'config': world}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'stage.launch.py')),
            condition=IfCondition(stage),
            launch_arguments={'one_tf_tree':one_tf_tree,
                              'enforce_prefixes':enforce_prefixes,
                              'world': world}.items()),
    ])
