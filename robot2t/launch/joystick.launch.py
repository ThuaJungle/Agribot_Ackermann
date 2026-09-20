import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'robot2t'
    
    # Đường dẫn file config
    joy_params = os.path.join(get_package_share_directory(pkg_name), 'config', 'joystick.yaml')

    # 1. Node đọc dữ liệu thô từ tay cầm (joy_node)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[{
            'dev': '/dev/input/js0', # Cổng tay cầm
            'deadzone': 0.05,        # Vùng chết để tránh cần gạt bị trôi
            'autorepeat_rate': 20.0,
        }]
    )

    # 2. Node chuyển đổi sang cmd_vel (teleop_node)
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joy_params,{'publish_stamped_twist': False}],
        remappings=[
            ('/cmd_vel', '/cmd_vel_teleop') # Đảm bảo topic khớp với robot
        ]
    )

    return LaunchDescription([
        joy_node,
        teleop_node
    ])