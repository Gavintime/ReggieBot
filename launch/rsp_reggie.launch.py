import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # use sim or real time based on param
    sim_mode = LaunchConfiguration('sim_mode')

    # read in the urdf file
    pkg_path = os.path.join(get_package_share_directory('reggiebot'))

    reggiebot_xacro_file = None
    if sim_mode:
        reggiebot_xacro_file = os.path.join(
            pkg_path, 'urdf', 'reggiebot_gazebo.urdf.xacro')
    else:
        reggiebot_xacro_file = os.path.join(
            pkg_path, 'urdf', 'reggiebot_physical.urdf.xacro')

    reggiebot_description = xacro.process_file(reggiebot_xacro_file)

    # create robot_state_publisher_node
    # the controller manager is started via the gazebo plugin in the xacro file
    params = {'robot_description': reggiebot_description.toxml(),
              'use_sim_time': sim_mode}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params])

    # launch nodes and set launch args
    return LaunchDescription([
        DeclareLaunchArgument('sim_mode',
                              default_value='false',
                              description='Use gazebo urdf and enable sim time if true'),
        robot_state_publisher])
