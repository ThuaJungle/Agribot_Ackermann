import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Lấy tên package và đường dẫn
    pkg_name = 'robot2t'
    pkg_share = get_package_share_directory(pkg_name)

    # 2. Đường dẫn đến file URDF/Xacro
    # Lưu ý: file xacro của bạn tên là 'robot.urdf.xacro' nằm trong folder 'description'
    default_model_path = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')

    # 3. Chuyển đổi Xacro thành URDF ngay khi chạy
    # Command này sẽ chạy lệnh 'xacro đường_dẫn_file'
    robot_description_content = Command(['xacro ', default_model_path])
    
    # Node 1: Robot State Publisher (Công bố hình dáng robot cho hệ thống)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # Node 2: Joint State Publisher GUI (Thanh trượt để bạn bẻ lái, xoay bánh thử)
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # Node 3: RViz2 (Phần mềm hiển thị 3D)
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # Trả về danh sách các node cần chạy
    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz
    ])
