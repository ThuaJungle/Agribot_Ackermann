import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from serial_motor_demo_msgs.msg import MotorCommand
from std_msgs.msg import Int16
import math

class AckermannAdapter(Node):
    def __init__(self):
        super().__init__('ackermann_adapter')

        # --- CẤU HÌNH XE ---
        self.wheel_radius = 0.075  # Bán kính bánh xe (mét) - Đã chốt ở bài trước
        self.steer_scale = 20.0    # Hệ số lái: 1.0 rad/s ở phím bấm = 20 độ Servo

        # Subscriber: Nghe lệnh từ bàn phím (/cmd_vel)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

        # Publisher: Gửi lệnh xuống Driver
        self.motor_pub = self.create_publisher(MotorCommand, 'motor_command', 10)
        self.steer_pub = self.create_publisher(Int16, 'steering_command', 10)
        
        self.get_logger().info("Ackermann Adapter Started. Ready for Teleop!")

    def cmd_vel_callback(self, msg):
        # 1. Xử lý Vận tốc (Linear X -> Motor Rad/s)
        v_linear = msg.linear.x  # m/s
        
        # Công thức: v = r * omega => omega = v / r
        w_wheel = v_linear / self.wheel_radius # rad/s
        
        motor_msg = MotorCommand()
        motor_msg.is_pwm = False # Chạy chế độ PID (Closed Loop)
        motor_msg.mot_1_req_rad_sec = float(w_wheel) # Bánh trái
        motor_msg.mot_2_req_rad_sec = float(w_wheel) # Bánh phải (Giả sử vi sai điện tử đơn giản)
        
        self.motor_pub.publish(motor_msg)

        # 2. Xử lý Góc lái (Angular Z -> Servo Degree)
        # Trong Teleop, Angular Z là tốc độ quay xe mong muốn. 
        # Ta map trực tiếp nó sang góc lái cho đơn giản.
        # Nhấn 'j' (Trái) -> angular.z dương -> Lái sang trái
        
        steering_val = msg.angular.z * self.steer_scale
        
        # Giới hạn góc lái
        steering_val = max(min(steering_val, 27.0), -27.0)

        steer_msg = Int16()
        steer_msg.data = int(steering_val)
        self.steer_pub.publish(steer_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AckermannAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()