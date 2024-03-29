import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory('reggiebot'))
    twist_mux_params_file = os.path.join(pkg_path, 'config', 'twist_mux.yaml')
    gazebo_params_file = os.path.join(pkg_path, 'config', 'gazebo_params.yaml')
    ekf_params_file = os.path.join(pkg_path, 'config', 'ekf.yaml')
    slam_params_file = os.path.join(pkg_path, 'config',
                                    'mapper_params_online_async.yaml')
    rviz_config_file = os.path.join(pkg_path, 'config', 'reggiebot.rviz')

    # multiplexes joypad, keyboard, and nav cmd_vels with a priority system
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        remappings={('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')},
        parameters=[twist_mux_params_file, {'use_sim_time': True}]
    )

    # reggie's rsp launch file
    rsp_reggie = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_path, 'launch', 'rsp_reggie.launch.py')]),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # launch gazebo sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'params_file': gazebo_params_file}.items()
    )

    # spawn reggie using it's desc from the rsp
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_reggiebot'],
        output='screen'
    )

    # delay starting controllers until the robot has been spawned in gazebo
    # TODO: fuse odom sensor data using robot_localization
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont']
    )
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=spawn_entity,
                                    on_exit=[diff_drive_spawner]))

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad']
    )
    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=spawn_entity,
                                    on_exit=[joint_broad_spawner]))

    imu_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_broad']
    )
    delayed_imu_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=spawn_entity,
                                    on_exit=[imu_broad_spawner]))

    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': True}]
    )

    # slam toolbox online async launch file
    # TODO: delay slam_toolbox until gazebo has finished loading
    # TODO: add option to just localize
    # (localization_launch.py in the slam_toolbox pkg,
    # it uses localization_slam_toolbox_node)
    # TODO: alternatively use map_server + amcl
    slam_toolbox = Node(
        parameters=[slam_params_file, {'use_sim_time': True}],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments={'-d': rviz_config_file}.items()
    )

    # launch
    return LaunchDescription(
        [twist_mux,
         rsp_reggie,
         gazebo,
         spawn_entity,
         delayed_diff_drive_spawner,
         delayed_joint_broad_spawner,
         delayed_imu_broad_spawner,
         slam_toolbox,
         robot_localization,
         rviz2
         ])
