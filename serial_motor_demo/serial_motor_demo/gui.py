import rclpy
from rclpy.node import Node
import time
from tkinter import *
import math

from serial_motor_demo_msgs.msg import MotorCommand
from serial_motor_demo_msgs.msg import MotorVels
from serial_motor_demo_msgs.msg import EncoderVals
from std_msgs.msg import Int16

class MotorGui(Node):

    def __init__(self):
        super().__init__('motor_gui')

        # Publisher cho Động cơ toc độ vi sai
        self.publisher = self.create_publisher(MotorCommand, 'motor_command', 10)

        # Publisher cho Hệ thống lái
        self.steer_pub = self.create_publisher(Int16, 'steering_command', 10)

        self.speed_sub = self.create_subscription(
            MotorVels,
            'motor_vels',
            self.motor_vel_callback,
            10)

        self.encoder_sub = self.create_subscription(
            EncoderVals,
            'encoder_vals',
            self.encoder_val_callback,
            10)

        self.tk = Tk()
        self.tk.title("Ackermann Robot Control")
        root = Frame(self.tk)
        root.pack(fill=BOTH, expand=True)

        Label(root, text="Ackermann Control Panel", font=("Arial", 14, "bold")).pack(pady=5)

        # --- KHUNG CHỌN CHẾ ĐỘ ---
        mode_frame = Frame(root, pady=5)
        mode_frame.pack(fill=X)
        self.mode_lbl = Label(mode_frame, text="ZZZZ")
        self.mode_lbl.pack(side=LEFT, padx=5)
        self.mode_btn = Button(mode_frame, text="ZZZZ", command=self.switch_mode)
        self.mode_btn.pack(expand=True)

        # --- KHUNG GIỚI HẠN TỐC ĐỘ ---
        slider_max_frame = Frame(root, pady=5)
        slider_max_frame.pack(fill=X)
        self.slider_max_label = Label(slider_max_frame, text="Max Rev/sec", state="disabled")
        self.slider_max_label.pack(side=LEFT, padx=5)
        self.slider_max_val_box = Entry(slider_max_frame, state="disabled", width=5)
        self.slider_max_val_box.pack(side=LEFT)
        self.max_val_update_btn = Button(slider_max_frame, text='Update', command=self.update_scale_limits, state="disabled")
        self.max_val_update_btn.pack(side=LEFT, padx=5)

        # --- KHUNG ĐIỀU KHIỂN LÁI (STEERING) ---
        steer_frame = Frame(root, borderwidth=2, relief="groove", pady=5)
        steer_frame.pack(fill=X, padx=5, pady=5)
        Label(steer_frame, text="STEERING (Stepper + Encoder)", fg="blue", font=("Arial", 10, "bold")).pack(side=TOP)

        # Slider Lái: Từ -28 độ đến 28 độ (Tự gửi lệnh khi kéo)
        self.steer_slider = Scale(steer_frame, from_=-28, to=28, orient=HORIZONTAL, length=300, command=self.send_steering_live)
        self.steer_slider.set(0) # Về giữa
        self.steer_slider.pack(fill=X, expand=True)
        
        Button(steer_frame, text="Center Wheel", command=self.center_steering).pack(pady=2)

        # --- KHUNG ĐỘNG CƠ ---
        motors_frame = Frame(root, borderwidth=2, relief="groove", pady=5)
        motors_frame.pack(fill=X, padx=5, pady=5)
        Label(motors_frame, text="TRACTION MOTORS", fg="red", font=("Arial", 10, "bold")).pack(side=TOP)

        m1_frame = Frame(motors_frame)
        m1_frame.pack(fill=X)
        Label(m1_frame, text="Motor 1").pack(side=LEFT)
        self.m1 = Scale(m1_frame, from_=-255, to=255, orient=HORIZONTAL)
        self.m1.pack(side=LEFT, fill=X, expand=True)

        m2_frame = Frame(motors_frame)
        m2_frame.pack(fill=X)
        Label(m2_frame, text="Motor 2").pack(side=LEFT)
        self.m2 = Scale(m2_frame, from_=-255, to=255, resolution=1, orient=HORIZONTAL)
        self.m2.pack(side=LEFT, fill=X, expand=True)
        self.m2.config(to=10)

        # --- CÁC NÚT ĐIỀU KHIỂN ---
        motor_btns_frame = Frame(root, pady=10)
        motor_btns_frame.pack()
        Button(motor_btns_frame, text='Send Once', command=self.send_motor_once, bg="#DDDDDD").pack(side=LEFT, padx=2)
        Button(motor_btns_frame, text='STOP ALL', command=self.stop_all, bg="red", fg="white", font=("Arial", 10, "bold")).pack(side=LEFT, padx=10)
        
        # --- KHUNG HIỂN THỊ THÔNG SỐ ---
        info_frame = Frame(root, pady=10)
        info_frame.pack(fill=X)

        enc_frame = Frame(info_frame)
        enc_frame.pack(fill=X)
        self.enc_lbl = Label(enc_frame, text="Encoders: ", width=10, anchor="w")
        self.enc_lbl.pack(side=LEFT)
        self.mot_1_enc_lbl = Label(enc_frame, text="0", width=10)
        self.mot_1_enc_lbl.pack(side=LEFT)
        self.mot_2_enc_lbl = Label(enc_frame, text="0", width=10)
        self.mot_2_enc_lbl.pack(side=LEFT)

        speed_frame = Frame(info_frame)
        speed_frame.pack(fill=X)
        self.spd_lbl = Label(speed_frame, text="Speed r/s: ", width=10, anchor="w")
        self.spd_lbl.pack(side=LEFT)
        self.mot_1_spd_lbl = Label(speed_frame, text="0.00", width=10)
        self.mot_1_spd_lbl.pack(side=LEFT)
        self.mot_2_spd_lbl = Label(speed_frame, text="0.00", width=10)
        self.mot_2_spd_lbl.pack(side=LEFT)

        self.set_mode(True)

    # --- Steering Control ---
    def send_steering_live(self, val):
        """Gửi lệnh lái ngay lập tức khi kéo thanh trượt"""
        msg = Int16()
        msg.data = int(val)
        self.steer_pub.publish(msg)

    def center_steering(self):
        """Đưa bánh về giữa"""
        self.steer_slider.set(0)
        # Hàm set sẽ tự kích hoạt send_steering_live nên không cần publish thủ công ở đây

    def stop_all(self):
        """Dừng xe và trả lái về giữa"""
        self.stop_motors()
        # self.center_steering() # Tùy chọn: trả lái về giữa khi dừng

    def show_values(self):
        print (self.m1.get(), self.m2.get())

    def send_motor_once(self):
        msg = MotorCommand()
        msg.is_pwm = self.pwm_mode
        if (self.pwm_mode):
            msg.mot_1_req_rad_sec = float(self.m1.get())
            msg.mot_2_req_rad_sec = float(self.m2.get())
        else:
            msg.mot_1_req_rad_sec = float(self.m1.get()*2*math.pi)
            msg.mot_2_req_rad_sec = float(self.m2.get()*2*math.pi)

        self.publisher.publish(msg)

    def stop_motors(self):
        msg = MotorCommand()
        msg.is_pwm = self.pwm_mode
        msg.mot_1_req_rad_sec = 0.0
        msg.mot_2_req_rad_sec = 0.0
        self.publisher.publish(msg)
        # Reset thanh trượt về 0
        self.m1.set(0)
        self.m2.set(0)

    def set_mode(self, new_mode):
        self.pwm_mode = new_mode
        if (self.pwm_mode):
            self.mode_lbl.config(text="Mode: PWM (Open Loop)")
            self.mode_btn.config(text="Switch to Feedback")
            self.slider_max_label.config(state="disabled")
            self.slider_max_val_box.config(state="disabled")
            self.max_val_update_btn.config(state="disabled")
        else:
            self.mode_lbl.config(text="Mode: PID (Closed Loop)")
            self.mode_btn.config(text="Switch to PWM")
            self.slider_max_label.config(state="normal")
            self.slider_max_val_box.config(state="normal")
            self.max_val_update_btn.config(state="normal")

        self.update_scale_limits()

    def motor_vel_callback(self, motor_vels):
        mot_1_spd_rev_sec = motor_vels.mot_1_rad_sec / (2*math.pi)
        mot_2_spd_rev_sec = motor_vels.mot_2_rad_sec / (2*math.pi)
        self.mot_1_spd_lbl.config(text=f"{mot_1_spd_rev_sec:.2f}")
        self.mot_2_spd_lbl.config(text=f"{mot_2_spd_rev_sec:.2f}")

    def encoder_val_callback(self, encoder_vals):
        self.mot_1_enc_lbl.config(text=f"{encoder_vals.mot_1_enc_val}")
        self.mot_2_enc_lbl.config(text=f"{encoder_vals.mot_2_enc_val}")

    def switch_mode(self):
        self.set_mode(not self.pwm_mode)

    def update_scale_limits(self):
        if (self.pwm_mode):
            self.m1.config(from_=-255, to=255, resolution=1)
            self.m2.config(from_=-255, to=255, resolution=1)
        else:
            try:
                lim = float(self.slider_max_val_box.get())
            except ValueError:
                lim = 10.0 # Giá trị mặc định nếu ô trống
            self.m1.config(from_=-lim, to=lim, resolution=0.1)
            self.m2.config(from_=-lim, to=lim, resolution=0.1)

    def update(self):
        self.tk.update()

def main(args=None):
    rclpy.init(args=args)
    motor_gui = MotorGui()
    rate = motor_gui.create_rate(20)    
    while rclpy.ok():
        rclpy.spin_once(motor_gui)
        motor_gui.update()
    motor_gui.destroy_node()
    rclpy.shutdown()