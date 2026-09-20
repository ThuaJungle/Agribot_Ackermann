#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <libserial/SerialPort.h>
#include <vector>
#include <string>
#include <cmath>
#include <sstream>
#include <chrono>

// --- THÔNG SỐ VẬT LÝ XE ---
#define WHEEL_BASE 0.38   
#define TRACK_WIDTH 0.51  
#define WHEEL_DIA 0.15    
#define ENC_CPR 1200.0    
#define LOOP_RATE 30.0    
#define SERVO_RATIO 1.0   

// Biến lưu trữ góc lái thực tế từ Arduino gửi lên
double last_known_steer_pos_ = 0.0;

namespace robot_control
{

class AckermannArduino : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
        return hardware_interface::CallbackReturn::ERROR;

    serial_port_name_ = info_.hardware_parameters["device"];
    baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
    
    hw_positions_.resize(info_.joints.size(), 0.0);
    hw_velocities_.resize(info_.joints.size(), 0.0);
    hw_commands_.resize(info_.joints.size(), 0.0);
    last_hw_positions_.resize(info_.joints.size(), 0.0);

    for (const auto & joint : info_.joints) {
        joint_names_.push_back(joint.name);
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    try {
      serial_conn_.Open(serial_port_name_);
      serial_conn_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
      rclcpp::sleep_for(std::chrono::seconds(2)); 
      last_read_time_ = rclcpp::Clock().now();
    } catch (...) {
      RCLCPP_ERROR(rclcpp::get_logger("AckermannArduino"), "Could not open serial port!");
      return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    serial_conn_.Close();
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      if (info_.joints[i].name.find("steer") != std::string::npos) {
         command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
      }
      else if (info_.joints[i].name.find("rear_axle") != std::string::npos) {
         command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
      }
    }
    return command_interfaces;
  }

  // --- HÀM READ ---
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    if (!serial_conn_.IsOpen()) return hardware_interface::return_type::ERROR;

    rclcpp::Time current_time = rclcpp::Clock().now();
    double dt = (current_time - last_read_time_).seconds();
    if (dt == 0) dt = 1e-9; 

    // Gửi lệnh hỏi Encoder
    serial_conn_.Write("e\r"); 

    // Đọc trong vòng lặp để lấy được cả lệnh e và l(nếu thay đổi góc lái)
    std::string line;
    long enc_left = 0, enc_right = 0;
    bool updated_encoders = false;
    int max_lines = 10; 

    while (max_lines > 0) 
    {
        try {
            // Timeout 10ms
            serial_conn_.ReadLine(line, '\n', 10); 
            
            if (line.empty()) continue;

            // TH1: GÓC LÁI (l <rad>) 
            if (line[0] == 'l') {
                try {
                    std::string val_str = line.substr(2);
                    last_known_steer_pos_ = std::stod(val_str); 
                } catch (...) {}
            }
            // TH2: ENCODER (e <trái> <phải>)
            else if (isdigit(line[0]) || line[0] == '-') {
                std::stringstream ss(line);
                if (ss >> enc_left >> enc_right) {
                    updated_encoders = true;
                }
            }
        } catch (...) { 
            break; 
        }
        max_lines--;
    }

    // Tính toán vị trí và vận tốc từ encoder
    double pos_left_rad = (enc_left / ENC_CPR) * 2 * M_PI;
    double pos_right_rad = (enc_right / ENC_CPR) * 2 * M_PI;

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        std::string name = joint_names_[i];

        if (name.find("steer") != std::string::npos) {
            //Dùng giá trị thực tế để cập nhật góc lái
            hw_positions_[i] = last_known_steer_pos_; 
            hw_velocities_[i] = 0.0; 
        }
        else if (name == "rear_axle_joint") {
            if (updated_encoders) {
                double avg_pos = (pos_left_rad + pos_right_rad) / 2.0;
                hw_positions_[i] = avg_pos;
                hw_velocities_[i] = (avg_pos - last_hw_positions_[i]) / dt;
                last_hw_positions_[i] = avg_pos;
            }
        }
        else if (name == "left_rear_wheel_joint") {
            if (updated_encoders) {
                hw_positions_[i] = pos_left_rad;
                hw_velocities_[i] = (pos_left_rad - last_hw_positions_[i]) / dt;
                last_hw_positions_[i] = pos_left_rad;
            }
        }
        else if (name == "right_rear_wheel_joint") {
            if (updated_encoders) {
                hw_positions_[i] = pos_right_rad;
                hw_velocities_[i] = (pos_right_rad - last_hw_positions_[i]) / dt;
                last_hw_positions_[i] = pos_right_rad;
            }
        }
    }

    last_read_time_ = current_time;
    return hardware_interface::return_type::OK;
  }

  // --- HÀM WRITE ---
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    if (!serial_conn_.IsOpen()) return hardware_interface::return_type::ERROR;

    double steer_cmd_rad = 0.0;
    double speed_cmd_rads = 0.0;

    for (size_t i = 0; i < info_.joints.size(); ++i) {
        if (joint_names_[i].find("steer") != std::string::npos) {
            steer_cmd_rad = hw_commands_[i];
        }
        if (joint_names_[i] == "rear_axle_joint") {
            speed_cmd_rads = hw_commands_[i];
        }
    }

    // Giới hạn góc lái
    if (steer_cmd_rad > 0.4713) steer_cmd_rad = 0.4713;
    if (steer_cmd_rad < -0.5411) steer_cmd_rad = -0.5411;

    // Tính toán vi sai cho model Ackermann
    double v_linear = speed_cmd_rads * (WHEEL_DIA / 2.0);
    double v_left = 0.0;
    double v_right = 0.0;

    if (std::abs(steer_cmd_rad) < 0.01) {
        v_left = v_linear;
        v_right = v_linear;
    } else {
        double tan_delta = std::tan(steer_cmd_rad);
        v_left = v_linear * (1.0 - (TRACK_WIDTH * tan_delta) / (2.0 * WHEEL_BASE));
        v_right = v_linear * (1.0 + (TRACK_WIDTH * tan_delta) / (2.0 * WHEEL_BASE));
    }

    double meters_to_ticks = (ENC_CPR / (M_PI * WHEEL_DIA)) * (1.0 / LOOP_RATE);
    int ticks_left = static_cast<int>(v_left * meters_to_ticks);
    int ticks_right = static_cast<int>(v_right * meters_to_ticks);
    int servo_val = static_cast<int>(steer_cmd_rad * 180.0 / M_PI * SERVO_RATIO);

    // --- GỬI LỆNH ĐIỀU KHIỂN ---
    std::string s_cmd = "s 0 " + std::to_string(servo_val) + "\r";
    serial_conn_.Write(s_cmd);

    std::string m_cmd = "m " + std::to_string(ticks_left) + " " + std::to_string(ticks_right) + "\r";
    serial_conn_.Write(m_cmd);

    return hardware_interface::return_type::OK;
  }

// Khai báo biến thành viên
private:
  LibSerial::SerialPort serial_conn_;
  std::string serial_port_name_;
  int baud_rate_;
  
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  std::vector<std::string> joint_names_;
  std::vector<double> last_hw_positions_;
  rclcpp::Time last_read_time_;
};

} // namespace
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robot_control::AckermannArduino, hardware_interface::SystemInterface)