import numpy as np

class DifferentialDriveRobot:
    def __init__(self, wheel_radius=0.12, wheel_distance=0.25, mass=1.2):
        self.r = wheel_radius
        self.D = wheel_distance
        self.M = mass 

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Lưu vận tốc HIỆN TẠI (Thực tế)
        self.v = 0.0 
        self.w = 0.0

        # GIẢ ĐỊNH ĐỘNG CƠ:
        # Lực đẩy tối đa của động cơ (Newton)
        self.MAX_FORCE = 5.0 
        # Mô-men xoắn xoay thân xe tối đa (N.m)
        self.MAX_TORQUE = 1.0 

    def update(self, wl, wr, dt):
        # 1. Tính vận tốc ĐÍCH (Target) mong muốn dựa trên input
        v_target = (self.r / 2) * (wr + wl)
        w_target = (self.r / self.D) * (wr - wl)

        # 2. Tính gia tốc tối đa cho phép dựa trên Khối lượng M
        # a = F / m
        # M càng lớn -> acc_linear càng nhỏ -> Tăng tốc càng chậm
        acc_linear_max = self.MAX_FORCE / self.M
        
        # Gia tốc góc: alpha = Torque / I (Mô-men quán tính)
        # Mô-men quán tính của hình hộp vuông cạnh D (Square Prism)
        # J = 1/6 * M * D^2
        I_robot = (1/6) * self.M * (self.D**2)
        acc_angular_max = self.MAX_TORQUE / I_robot

        # 3. Cập nhật Vận tốc dài (v) tiến dần tới v_target
        # Nếu đang chậm hơn đích -> Tăng tốc
        if self.v < v_target:
            self.v += acc_linear_max * dt
            if self.v > v_target: self.v = v_target # Không vượt quá
        # Nếu đang nhanh hơn đích -> Giảm tốc
        elif self.v > v_target:
            self.v -= acc_linear_max * dt
            if self.v < v_target: self.v = v_target

        # 4. Cập nhật Vận tốc góc (w) tương tự
        if self.w < w_target:
            self.w += acc_angular_max * dt
            if self.w > w_target: self.w = w_target
        elif self.w > w_target:
            self.w -= acc_angular_max * dt
            if self.w < w_target: self.w = w_target

        # 5. Cập nhật vị trí (Kinematics) dùng vận tốc ĐÃ CÓ QUÁN TÍNH
        self.x += self.v * np.cos(self.theta) * dt
        self.y += self.v * np.sin(self.theta) * dt
        self.theta += self.w * dt

        return self.x, self.y, self.theta

    def params(self):
        return self.D, self.r