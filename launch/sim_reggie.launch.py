import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory('reggie_bot'))

    # reggie's rsp launch file
    rsp_reggie_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_path, 'launch'), '/rsp_reggie.launch.py']),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # launch gazebo sim
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    )

    # spawn reggie using it's desc from the rsp
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_reggiebot'],
                        output='screen')

    # TODO: wait for spawn entity to finish
    diff_drive_spawner = Node(package='controller_manager',
                              executable='spawner',
                              arguments=['diff_cont'])

    joint_broad_spawner = Node(package='controller_manager',
                              executable='spawner',
                              arguments=['joint_broad'])

    # launch
    return LaunchDescription([rsp_reggie_launch,
                              gazebo_launch,
                              spawn_entity,
                              diff_drive_spawner,
                              joint_broad_spawner])
