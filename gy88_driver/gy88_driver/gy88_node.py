#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from smbus2 import SMBus
import math
import time

# --- CẤU HÌNH ĐỊA CHỈ ---
MPU6050_ADDR = 0x68
HMC5883L_ADDR = 0x1E

# Registers
PWR_MGMT_1   = 0x6B
INT_PIN_CFG  = 0x37
ACCEL_CONFIG = 0x1C
GYRO_CONFIG  = 0x1B
ACCEL_XOUT_H = 0x3B

MAG_CONFIG_A  = 0x00
MAG_CONFIG_B  = 0x01
MAG_MODE      = 0x02
MAG_DATA_X_H  = 0x03

class GY88Node(Node):
    def __init__(self):
        super().__init__('gy88_node')
        
        # --- THAM SỐ ---
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('pub_rate', 30.0)

        self.i2c_bus_id = self.get_parameter('i2c_bus').value
        self.frame_id = self.get_parameter('frame_id').value
        self.rate = self.get_parameter('pub_rate').value

        # --- PUBLISHER ---
        # IMU Raw: Chứa Accel + Gyro (Chưa có Orientation)
        # Đổi tên thành 'data_raw' để đúng chuẩn input cho Madgwick Filter
        self.imu_pub_ = self.create_publisher(Imu, '/imu/data_raw', 10)
        
        # Mag: Chứa dữ liệu từ trường
        self.mag_pub_ = self.create_publisher(MagneticField, '/imu/mag', 10)
        
        self.offset = {'ax':0.0, 'ay':0.0, 'az':0.0, 'gx':0.0, 'gy':0.0, 'gz':0.0}

        try:
            self.bus = SMBus(self.i2c_bus_id)
            self.init_mpu6050()
            self.init_hmc5883l()
            self.calibrate_mpu6050()
        except Exception as e:
            self.get_logger().error(f"FATAL I2C Error: {e}")
            return

        self.timer = self.create_timer(1.0/self.rate, self.timer_callback)
        self.get_logger().info(f"GY-88 Raw Driver Started (Freq: {self.rate}Hz)")

    def init_mpu6050(self):
        self.bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        self.bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, 0x08)
        self.bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, 0x08)
        self.bus.write_byte_data(MPU6050_ADDR, INT_PIN_CFG, 0x02) # Bypass ON
        time.sleep(0.1)

    def init_hmc5883l(self):
        self.bus.write_byte_data(HMC5883L_ADDR, MAG_CONFIG_A, 0x70)
        self.bus.write_byte_data(HMC5883L_ADDR, MAG_CONFIG_B, 0x20)
        self.bus.write_byte_data(HMC5883L_ADDR, MAG_MODE, 0x00)

    def read_mpu_raw(self):
        try:
            data = self.bus.read_i2c_block_data(MPU6050_ADDR, ACCEL_XOUT_H, 14)
            def parse(idx):
                val = (data[idx] << 8) | data[idx+1]
                return val - 65536 if val >= 32768 else val
            return {
                'ax': parse(0), 'ay': parse(2), 'az': parse(4),
                'gx': parse(8), 'gy': parse(10), 'gz': parse(12)
            }
        except OSError:
            return None

    def read_mag_raw(self):
        try:
            data = self.bus.read_i2c_block_data(HMC5883L_ADDR, MAG_DATA_X_H, 6)
            def parse(idx):
                val = (data[idx] << 8) | data[idx+1]
                return val - 65536 if val >= 32768 else val
            # HMC5883L Order: X, Z, Y
            return parse(0), parse(4), parse(2) 
        except OSError:
            return 0, 0, 0

    def calibrate_mpu6050(self):
        self.get_logger().info("Calibrating... Keep Still!")
        samples = 50
        sums = {'ax':0, 'ay':0, 'az':0, 'gx':0, 'gy':0, 'gz':0}
        count = 0
        for _ in range(samples):
            raw = self.read_mpu_raw()
            if raw:
                for k in sums: sums[k] += raw[k]
                count += 1
            time.sleep(0.01)
        
        if count > 0:
            self.offset['ax'] = sums['ax'] / count
            self.offset['ay'] = sums['ay'] / count
            self.offset['az'] = (sums['az'] / count) - 8192
            self.offset['gx'] = sums['gx'] / count
            self.offset['gy'] = sums['gy'] / count
            self.offset['gz'] = sums['gz'] / count
            self.get_logger().info("Calibration Done.")

    def timer_callback(self):
        imu_raw = self.read_mpu_raw()
        mag_x, mag_y, mag_z = self.read_mag_raw()
        
        if imu_raw is None: return

        now = self.get_clock().now().to_msg()
        
        # --- CONSTANTS ---
        ACCEL_SCALE = 8192.0
        GYRO_SCALE = 65.5
        MAG_SCALE = 1090.0
        G = 9.81
        RAD = math.pi / 180.0

        # --- IMU MESSAGE (Raw) ---
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = self.frame_id
        
        imu_msg.linear_acceleration.x = ((imu_raw['ax'] - self.offset['ax']) / ACCEL_SCALE) * G
        imu_msg.linear_acceleration.y = ((imu_raw['ay'] - self.offset['ay']) / ACCEL_SCALE) * G
        imu_msg.linear_acceleration.z = ((imu_raw['az'] - self.offset['az']) / ACCEL_SCALE) * G

        imu_msg.angular_velocity.x = ((imu_raw['gx'] - self.offset['gx']) / GYRO_SCALE) * RAD
        imu_msg.angular_velocity.y = ((imu_raw['gy'] - self.offset['gy']) / GYRO_SCALE) * RAD
        imu_msg.angular_velocity.z = ((imu_raw['gz'] - self.offset['gz']) / GYRO_SCALE) * RAD

        # Đánh dấu "Không có orientation" để Madgwick tự tính
        imu_msg.orientation_covariance[0] = -1.0 
        
        self.imu_pub_.publish(imu_msg)

        # --- MAG MESSAGE ---
        mag_msg = MagneticField()
        mag_msg.header.stamp = now
        mag_msg.header.frame_id = self.frame_id
        
        mag_msg.magnetic_field.x = (mag_x / MAG_SCALE) * 1e-4
        mag_msg.magnetic_field.y = (mag_y / MAG_SCALE) * 1e-4
        mag_msg.magnetic_field.z = (mag_z / MAG_SCALE) * 1e-4
        mag_msg.magnetic_field_covariance = [1e-6, 0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 0.0, 1e-6]

        self.mag_pub_.publish(mag_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GY88Node()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()