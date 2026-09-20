import os
from ament_index_python.packages import get_package_share_directory
import controller_manager
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'robot2t' # Tên package của bạn
    pkg_share = get_package_share_directory(pkg_name)

    ####################################[SENSOR]###################################
    # 1. IMU Driver (GY-88) - Node bạn đã viết
    # LƯU Ý: Hãy sửa code Python driver để publish ra topic '/imu/data_raw' 
    # thay vì '/imu/data' để đúng chuẩn input của Madgwick.
    gy88_driver = Node(
        package='gy88_driver',
        executable='gy88_node',
        name='gy88_driver_node',
        parameters=[{'pub_rate': 30.0}]
    )

    # 2. Madgwick Filter (C++ chuẩn ROS)
    madgwick_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[{
            'use_mag': True,        # Dùng La bàn để sửa hướng (cực quan trọng)
            'publish_tf': False,    # Để Robot_Localization (EKF) lo phần TF
            'world_frame': 'enu',   # Chuẩn Đông-Bắc-Lên
            'fixed_frame': 'odom',  # Hoặc 'map' tùy cấu trúc TF của bạn
            'gain': 0.1,            # Hệ số tin tưởng (giống beta trong code Python)
            'zeta': 0.0,            # Gyro drift gain (thường để 0 nếu gyro tốt)
            'mag_bias_x': 0.0,      # Nếu bạn chưa calibrate la bàn kỹ
            'mag_bias_y': 0.0,
            'mag_bias_z': 0.0
        }],
        remappings=[
            # Output đã lọc (đưa vào EKF)
            ('/imu/data', '/imu/data_filtered')
        ]
    )
    ###############################################################################################
    # NODE EKF (Bộ lọc Kalman)
    ekf_config = os.path.join(get_package_share_directory(pkg_name), 'config', 'ekf.yaml')
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[('/odometry/filtered', '/odom_filtered')] 
    )
    ###############################################################################

    # Launch!
    return LaunchDescription([
        gy88_driver,
        madgwick_filter,
        ekf_node
    ])