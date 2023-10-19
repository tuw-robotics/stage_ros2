# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    this_directory = get_package_share_directory('stage_ros2')

    # Create the launch configuration variables

    # Declare the launch arguments
    namespace = LaunchConfiguration('namespace')
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='robot_0',
        description=('Top-level namespace. The value will be used to replace the '
                     '<robot_namespace> keyword on the rviz config file.'))

    one_tf_tree = LaunchConfiguration('one_tf_tree')
    one_tf_tree_cmd = DeclareLaunchArgument(
        'one_tf_tree',
        default_value='false',
        description='on true all tfs are published with a namespace on /tf and /tf_static')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo/Stage) clock if true')
    
    rviz_config = LaunchConfiguration('config')
    rviz_config_arg = DeclareLaunchArgument(
        'config',
        default_value=TextSubstitution(text='empty'),
        description='Use empty, cave or roblab to load a TUW enviroment')
    
    def rviz_launch_configuration(context):
        file = os.path.join(
            this_directory,
            'config/rviz',
            context.launch_configurations['config'] + '.rviz')
        return [SetLaunchConfiguration('config', file)]

    rviz_launch_configuration_arg = OpaqueFunction(function=rviz_launch_configuration)

    # Launch rviz
    start_rviz_one_tf_tree_cmd = Node(
        condition=IfCondition(one_tf_tree),
        package='rviz2',
        executable='rviz2',
        namespace=namespace,
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{
                "use_sim_time": use_sim_time}])

    start_rviz_multiple_tf_trees_cmd = Node(
        condition=UnlessCondition(one_tf_tree),
        package='rviz2',
        executable='rviz2',
        namespace=namespace,
        arguments=['-d', rviz_config, '-t', '{NAMESPACE} - {CONFIG_PATH}/{CONFIG_FILENAME} - RViz2'],
        output='screen',
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('/camera_info', 'camera_info'),
                    ('/goal_pose', 'goal_pose'),
                    ('/clicked_point', 'clicked_point'),
                    ('/initialpose', 'initialpose')],
        parameters=[{
                "use_sim_time": use_sim_time}])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(one_tf_tree_cmd)
    ld.add_action(rviz_config_arg)
    ld.add_action(rviz_launch_configuration_arg)

    # Add any conditioned actions
    ld.add_action(start_rviz_one_tf_tree_cmd)
    ld.add_action(start_rviz_multiple_tf_trees_cmd)

    return ld
