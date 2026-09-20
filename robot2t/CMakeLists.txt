cmake_minimum_required(VERSION 3.8)
project(robot2t)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. TÌM GÓI (Thêm PkgConfig)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(PkgConfig REQUIRED) 
find_package(sensor_msgs REQUIRED)

# 2. Dùng PkgConfig để tìm libserial
pkg_check_modules(SERIAL_LIB REQUIRED libserial)

# 3. TẠO THƯ VIỆN
add_library(ackermann_arduino SHARED
  src/ackermann_arduino.cpp
)

# 4. LIÊN KẾT THƯ VIỆN (Sửa đoạn này)
target_include_directories(ackermann_arduino PUBLIC ${SERIAL_LIB_INCLUDE_DIRS})
target_link_libraries(ackermann_arduino ${SERIAL_LIB_LIBRARIES}) # Link file .so tìm được

# 5. LIÊN KẾT ROS DEPENDENCIES
ament_target_dependencies(ackermann_arduino
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  sensor_msgs
)

# 6. XUẤT PLUGIN
pluginlib_export_plugin_description_file(hardware_interface robot_control.xml)

# 7. CÀI ĐẶT (INSTALL)
install(
  TARGETS ackermann_arduino
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES robot_control.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch meshes description config maps worlds
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()