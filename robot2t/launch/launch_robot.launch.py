import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'robot2t' # Tên package của bạn

    # 1. Khởi động Robot State Publisher (Đọc URDF)
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(pkg_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
    )

    # 2. Cấu hình Controller Manager
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    # File yaml chứa cấu hình controller
    controller_params_file = os.path.join(get_package_share_directory(pkg_name),'config','my_controllers.yaml')

    # Node quản lý Controller (Đây là trái tim của ros2_control)
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory(pkg_name), 'description', 'robot.urdf.xacro')])},
                    controller_params_file],
        remappings=[
            ('/robot_controller/reference', '/cmd_vel'),
            ('/robot_controller/odometry', '/odom'),
            ('/robot_controller/tf_odometry', '/tf')
        ]
    )

    # 3. Spawn các Controller (Kích hoạt sau khi manager chạy)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    ackermann_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_controller"], # Tên controller trong file yaml
    )

    #steering_spawner = Node(
    #    package="controller_manager",
    #    executable="spawner",
    #    arguments=["steering_controller"],
    #)

    # Trì hoãn việc spawn controller để đợi manager sẵn sàng
    delayed_ackermann_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[ackermann_spawner],
        )
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    # Launch!
    return LaunchDescription([
        rsp,
        controller_manager,
        delayed_ackermann_spawner,
        delayed_joint_broad_spawner,
    ])