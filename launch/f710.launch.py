import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    this_directory = get_package_share_directory('stage_ros2')
    joy_config = LaunchConfiguration('joy_config')
    joy_dev = LaunchConfiguration('joy_dev')
    config_filepath = LaunchConfiguration('config_filepath')
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value=TextSubstitution(text='robot_0'))

    return LaunchDescription([
        namespace_arg,
        DeclareLaunchArgument('joy_config', default_value='f710'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        DeclareLaunchArgument('config_filepath', default_value=[
            TextSubstitution(text=os.path.join(this_directory, 'config', 'teleop_twist_joy', '')),
            joy_config, TextSubstitution(text='.config.yaml')]),

        Node(
            namespace=LaunchConfiguration('namespace'),
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }]),
        Node(
            namespace=LaunchConfiguration('namespace'),
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy_node', parameters=[config_filepath]),
    ])
