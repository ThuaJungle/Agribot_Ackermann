import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class CmdVelConverter(Node):
    def __init__(self):
        super().__init__('cmd_vel_converter')
        
        # 1. Nghe lệnh từ Teleop (Twist thường)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_teleop', # Teleop sẽ gửi vào đây
            self.listener_callback,
            10)
            
        # 2. Gửi lệnh cho Controller (TwistStamped)
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)

    def listener_callback(self, msg):
        # Tạo gói tin TwistStamped mới
        stamped_msg = TwistStamped()
        
        # Gán thời gian hiện tại
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.header.frame_id = 'base_link' # Khung tham chiếu
        
        # Copy dữ liệu điều khiển sang
        stamped_msg.twist = msg
        
        # Gửi đi
        self.publisher.publish(stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()