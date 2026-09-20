import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serial_motor_demo',
            executable='driver',
            name='serial_motor_driver',
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyUSB0'}, 
                {'baud_rate': 115200},
                {'loop_rate': 30},
                {'encoder_cpr': 1200}
            ]
        )
    ])
