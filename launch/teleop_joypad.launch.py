import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    joy_config = LaunchConfiguration('joy_config')
    joy_dev = LaunchConfiguration('joy_dev')
    config_filepath = LaunchConfiguration('config_filepath')
    cmd_vel = LaunchConfiguration('cmd_vel')

    joy_node = Node(package='joy', executable='joy_node', name='joy_node',
                    parameters=[{
                        'dev': joy_dev,
                        'deadzone': 0.3,
                        'autorepeat_rate': 20.0,
                    }])

    teleop_twist_joy_node = Node(package='teleop_twist_joy',
                                 executable='teleop_node',
                                 name='teleop_twist_joy_node',
                                 parameters=[config_filepath],
                                 remappings=[('/cmd_vel', cmd_vel)])

    return LaunchDescription([
        DeclareLaunchArgument('joy_config', default_value='xbox'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        DeclareLaunchArgument('config_filepath', default_value=[
            TextSubstitution(text=os.path.join(
                get_package_share_directory('teleop_twist_joy'), 'config', '')),
            joy_config, TextSubstitution(text='.config.yaml')]),
        DeclareLaunchArgument('cmd_vel', default_value='/cmd_vel'),

        joy_node,
        teleop_twist_joy_node,
    ])
