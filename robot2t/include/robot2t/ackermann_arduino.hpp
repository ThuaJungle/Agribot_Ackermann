#ifndef ROBOT2T__ROBOT_SYSTEM_HPP_
#define ROBOT2T__ROBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

// Các thư viện ROS 2 Control
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Thư viện Serial Port
#include <libserial/SerialPort.h>

namespace robot2t_hardware
{
class RobotSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RobotSystem)

  // --- 1. Khởi tạo (Chuẩn mới của ROS 2 Jazzy/Rolling) ---
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  // --- 2. Cấu hình (Khi node chuyển sang state Inactive) ---
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  // --- 3. Kích hoạt (Khi node chuyển sang state Active - Robot bắt đầu chạy) ---
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  // --- 4. Ngưng kích hoạt (Khi node dừng lại) ---
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // --- 5. Export Interfaces (Bắt buộc phải có để tránh lỗi build) ---
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // --- 6. Vòng lặp Read/Write (Chạy liên tục thời gian thực) ---
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Đối tượng Serial Port
  std::unique_ptr<LibSerial::SerialPort> serial_port_;
  
  // Tên cổng COM (ví dụ: /dev/ttyUSB0)
  std::string device_name_;

  // Các biến lưu trữ giá trị Command (Gửi xuống) và State (Đọc lên)
  // Dùng vector để tự động thích ứng với số lượng bánh xe
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
};

}  // namespace robot2t_hardware

#endif  // ROBOT2T__ROBOT_SYSTEM_HPP_