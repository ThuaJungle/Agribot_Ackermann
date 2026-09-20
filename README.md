################### Connecting Raspberry Pi 5 ##################
#Vào thư mục của Pi 5
ketnoipi
ssh robot2t@robot2t.local
################ Khởi động hardware interfaces #################
cd ~/robot_ws/
source install/setup.bash
ros2 launch robot2t launch_robot.launch.py
################### Khởi động Lidar ############################
cd ~/ros2_ws
source install/setup.bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
################### Khởi động Lidar ############################
cd ~/sllidar_ws
source install/setup.bash
ros2 launch sllidar_ros2 sllidar_c1_launch.py
################### Khởi động cảm biến #########################
cd ~/robot_ws/
source install/setup.bash
ros2 launch robot2t gps_fusion.launch.py
################### Remap đúng topic để điều khiển ###################
cd ~/devPC_ws/
source install/setup.bash
python3 ~/devPC_ws/src/robot2t/launch/cmd_vel_converter.py
################# NAVIGATION 2 CONTROL ########################
cd ~/robot_ws
source install/setup.bash
ros2 launch robot2t navigation_nomap.launch.py
################# Lệnh chạy bao quát ##########################
cd ~/robot_ws/
source install/setup.bash
python3 ~/robot_ws/src/robot2t/launch/start_coverage.py
################# Lệnh chạy bao quát ##########################
cd ~/robot_ws/
source install/setup.bash
python3 ~/robot_ws/src/robot2t/launch/f2c_launch.py
#Kích hoạt Coverage Server để Navigation không bị crash
ros2 lifecycle set /coverage_server configure
ros2 lifecycle set /coverage_server activate
################### Khởi động SLAM TOOLBOX ####################
cd ~/robot_ws
source install/setup.bash
ros2 launch robot2t slam.launch.py
################# Khởi động RVIZ2 #############################
cd ~/devPC_ws/
source install/setup.bash
ros2 run rviz2 rviz2
############## Cấu hình sau khi khởi động SLAM TOOLBOX ########
#Cho phép cấu hình slam toolbox
ros2 lifecycle set /slam_toolbox configure
#kích hoạt slam toolbox
ros2 lifecycle set /slam_toolbox activate
################# TELEOPERATED CONTROL ########################
#Khởi động bàn phím điều khiển Remap để gửi vào /cmd_vel_teleop thay vì /cmd_vel
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_teleop
###############################################################
#Khởi động tay cầm Nova Lite2 (SONY) để điều khiển
cd ~/robot_ws
source install/setup.bash
ros2 launch robot2t joystick.launch.py
#Chạy có map
ros2 launch robot2t navigation.launch.py
#lưu map
ros2 run nav2_map_server map_saver_cli -f ~/robot_ws/src/robot2t/maps/my_map
################# Others Configure ###########################
sudo chmod 666 /dev/i2c-1
ros2 run gy88_driver gy88_node
