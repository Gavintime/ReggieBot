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

    # reggie's rsp launch file
    rsp_reggie_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_path, 'launch'), '/rsp_reggie.launch.py']),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # launch gazebo sim
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            # 'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
            'params_file': gazebo_params_file
        }.items()
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

    # delay starting controllers until the robot has been spawned in gazebo
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=spawn_entity,
                                    on_exit=[diff_drive_spawner]))

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=spawn_entity,
                                    on_exit=[joint_broad_spawner]))

    # Replace the diff_drive_spawner in the final return with delayed_diff_drive_spawner

    # launch
    return LaunchDescription([rsp_reggie_launch,
                              gazebo_launch,
                              spawn_entity,
                              delayed_diff_drive_spawner,
                              delayed_joint_broad_spawner])
