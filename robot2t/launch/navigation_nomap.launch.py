import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('robot2t')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # 1. Đường dẫn file Params (YAML)
    params_file = os.path.join(pkg_share, 'config', 'nav2_no_map_params.yaml')
    
    # 2. Đường dẫn file Behavior Tree (XML)
    # Logic Ackermann
    # bt_xml_file = os.path.join(pkg_share, 'config', 'navigation_ackermann.xml')
    
    # 3. Đường dẫn Map
    map_file = os.path.join(pkg_share, 'maps', 'my_new_map.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'params_file': params_file,
                'use_sim_time': 'False', # False cho xe thật
                'log_level': 'info',
                # Thay thế file BT mặc định bằng file BT Ackermann
                # 'default_nav_to_pose_bt_xml': bt_xml_file,
                # 'default_nav_through_poses_bt_xml': bt_xml_file, 
                # -------------------------------------
            }.items()
        )
    ])