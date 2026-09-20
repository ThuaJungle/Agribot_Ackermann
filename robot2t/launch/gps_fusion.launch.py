import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_path = get_package_share_directory('robot2t')
    gy88_pkg = get_package_share_directory('gy88_driver')
    ekf_config_path = os.path.join(pkg_path, 'config', 'dual_ekf.yaml')

    # IMU Driver (GY-88)
    # Node publish /imu/data_raw và /imu/mag
    imu_driver_node = Node(
        package='gy88_driver',
        executable='gy88_node',
        name='gy88_driver',
        parameters=[{'pub_rate': 30.0}]
    )

    # Madgwick Filter (Tạo Orientation từ Raw IMU)
    # Input: /imu/data_raw + /imu/mag -> Output: /imu/data
    madgwick_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        parameters=[{
            'use_mag': True,
            'publish_tf': False,
            'world_frame': 'enu'
        }],
        remappings=[
            ('/imu/data_raw', '/imu/data_raw'),
            ('/imu/mag', '/imu/mag'),
            ('/imu/data', '/imu/data')
        ]
    )

    # 3. GPS Driver (NMEA)
    # Đọc từ cổng serial GPS
    gps_driver_node = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='gps_driver',
        output='screen',
        parameters=[{
            'port': '/dev/ttyAMA0',
            'baud': 38400,       
            'frame_id': 'gps_link'
        }]
    )

    # 4. Navsat Transform Node (Cầu nối GPS -> EKF)
    # Cần: /imu/data, /gps/fix, /odometry/filtered
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[
            ('imu/data', '/imu/data'),
            ('gps/fix', '/fix'), 
            ('odometry/filtered', '/odometry/global') # Dùng EKF Global để làm mốc
        ]
    )

    # 5. EKF Local (Odom)
    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[('odometry/filtered', '/odometry/local')]
    )

    # 6. EKF Global (Map)
    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[('odometry/filtered', '/odometry/global')]
    )

    return LaunchDescription([
        imu_driver_node,
        madgwick_filter,
        gps_driver_node,
        navsat_transform_node,
        ekf_local_node,
        ekf_global_node
    ])