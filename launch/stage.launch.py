#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    this_directory = get_package_share_directory('stage_ros2')

    stage_world_arg = DeclareLaunchArgument(
        'world',
        default_value=TextSubstitution(text='cave'),
        description='World file relative to the project world file, without .world')


    enforce_prefixes = LaunchConfiguration('enforce_prefixes')
    enforce_prefixes_arg = DeclareLaunchArgument(
        'enforce_prefixes',
        default_value='false',
        description='on true a prefixes are used for a single robot environment')
    

    one_tf_tree = LaunchConfiguration('one_tf_tree')
    one_tf_tree_arg = DeclareLaunchArgument(
        'one_tf_tree',
        default_value='false',
        description='on true all tfs are published with a namespace on /tf and /tf_static')
    
    def stage_world_configuration(context):
        file = os.path.join(
            this_directory,
            'world',
            context.launch_configurations['world'] + '.world')
        return [SetLaunchConfiguration('world_file', file)]

    stage_world_configuration_arg = OpaqueFunction(function=stage_world_configuration)

    return LaunchDescription([
        stage_world_arg,
        one_tf_tree_arg, 
        enforce_prefixes_arg, 
        stage_world_configuration_arg,
        Node(
            package='stage_ros2',
            executable='stage_ros2',
            name='stage',
            parameters=[{'one_tf_tree': one_tf_tree,
                        'enforce_prefixes': enforce_prefixes,
                "world_file": [LaunchConfiguration('world_file')]}],
        )
    ])
