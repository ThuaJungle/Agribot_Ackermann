import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_name = 'robot2t'
    pkg_share = get_package_share_directory(pkg_name)
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    map_file = '/home/robot2t/robot_ws/src/robot2t/maps/my_new_map.yaml'
    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'params_file': params_file,
                'use_sim_time': 'False',
                'log_level': 'info'
            }.items()
        )
    ])