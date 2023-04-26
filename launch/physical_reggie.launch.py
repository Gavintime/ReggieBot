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
    ekf_params_file = os.path.join(pkg_path, 'config', 'ekf.yaml')
    slam_params_file = os.path.join(pkg_path, 'config',
                                    'mapper_params_online_async.yaml')
    rviz_config_file = os.path.join(pkg_path, 'config', 'reggiebot.rviz')
    reggiebot_xacro_file = os.path.join(
        pkg_path, 'urdf', 'reggiebot_physical.urdf.xacro')
    controler_params_file = os.path.join(
        pkg_path, 'config', 'controller params.yaml')

    # multiplexes joypad, keyboard, and nav cmd_vels with a priority system
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        remappings={('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')},
        parameters=[twist_mux_params_file, {'use_sim_time': False}]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": reggiebot_xacro_file},
                    controler_params_file],
        output="both",
    )

    # reggie's rsp launch file
    rsp_reggie = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_path, 'launch', 'rsp_reggie.launch.py')]),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    # TODO: fuse odom sensor data using robot_localization
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont']
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad']
    )

    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': False}]
    )

    # slam toolbox online async launch file
    # TODO: add option to just localize
    # (localization_launch.py in the slam_toolbox pkg,
    # it uses localization_slam_toolbox_node)
    # TODO: alternatively use map_server + amcl
    # slam_toolbox = Node(
    #     parameters=[slam_params_file, {'use_sim_time': False}],
    #     package='slam_toolbox',
    #     executable='async_slam_toolbox_node',
    #     name='slam_toolbox',
    #     output='screen'
    # )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments={'-d': rviz_config_file}.items()
    )

    # launch
    return LaunchDescription(
        [twist_mux,
         control_node,
         rsp_reggie,
         diff_drive_spawner,
         joint_broad_spawner,
         #  slam_toolbox,
         robot_localization,
         rviz2
         ])
