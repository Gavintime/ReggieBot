import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # use sim or real time based on param
    use_sim_time = LaunchConfiguration('use_sim_time')

    # read in the urdf file
    pkg_path = os.path.join(get_package_share_directory('reggie_bot'))
    reggiebot_xacro_file = os.path.join(
        pkg_path, 'urdf', 'reggiebot.urdf.xacro')
    reggiebot_description = xacro.process_file(reggiebot_xacro_file)

    # create robot_state_publisher_node
    # the controller manager is started via the gazebo plugin in the xacro file
    params = {'robot_description': reggiebot_description.toxml(),
              'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params])

    # launch nodes and set launch args
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',
                              default_value='false',
                              description='Use sim time if true'),
        node_robot_state_publisher])
