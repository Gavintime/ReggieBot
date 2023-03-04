from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='rplidar_ros',
             name='rplidar_composition',
             executable='rplidar_composition',
             output='screen',
             parameters=[{
                 'serial_port': '/dev/rplidar-a1',
                 'serial_baudrate': 115200,
                 'frame_id': 'laser_frame',
                 'inverted': False,
                 'angle_compensate': True,
                 'scan_mode': 'Standard'
                 #  'scan_mode': 'Express'
             }]
             )
    ])
