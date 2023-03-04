import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory('reggie_bot'))
    gazebo_params_file = os.path.join(pkg_path, 'config', 'gazebo_params.yaml')
    slam_params_file = os.path.join(pkg_path, 'config',
                                    'mapper_params_online_async.yaml')
    rviz_config_file = os.path.join(pkg_path, 'config', 'reggiebot.rviz')

    # reggie's rsp launch file
    rsp_reggie_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_path, 'launch', 'rsp_reggie.launch.py')]),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # launch gazebo sim
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'params_file': gazebo_params_file}.items()
    )

    # spawn reggie using it's desc from the rsp
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_reggiebot'],
                        output='screen')

    # delay starting controllers until the robot has been spawned in gazebo
    diff_drive_spawner = Node(package='controller_manager',
                              executable='spawner',
                              arguments=['diff_cont'])
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=spawn_entity,
                                    on_exit=[diff_drive_spawner]))

    joint_broad_spawner = Node(package='controller_manager',
                               executable='spawner',
                               arguments=['joint_broad'])
    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=spawn_entity,
                                    on_exit=[joint_broad_spawner]))

    # slam toolbox online async launch file
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
            'slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={'use_sim_time': 'true',
                          'slam_params_file': slam_params_file}.items()
    )

    rviz2_node = Node(package='rviz2', executable='rviz2',
                      name='rviz2', arguments={'-d': rviz_config_file}.items())

    # launch
    return LaunchDescription([rsp_reggie_launch,
                              gazebo_launch,
                              spawn_entity,
                              delayed_diff_drive_spawner,
                              delayed_joint_broad_spawner,
                              slam_toolbox_launch,
                              rviz2_node])
