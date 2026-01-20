import tkinter as tk
from tkinter import ttk, messagebox
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
import serial
import serial.tools.list_ports
import threading
import time

# Import file module
from kinematics import DifferentialDriveRobot
from robot_draw import draw_robot

# --- MÀU SẮC ---
BG_COLOR = "white"
PANEL_BG = "#f5f5f5"

class RobotGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot 2 Bánh")
        self.root.state('zoomed') # Mở toàn màn hình
        self.root.configure(bg=BG_COLOR)

        # --- TẠO THANH CUỘN (SCROLLBAR) ---
        # 1. Canvas chính
        self.main_canvas = tk.Canvas(root, bg=BG_COLOR)
        self.main_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # 2. Scrollbar dọc
        self.scrollbar = ttk.Scrollbar(root, orient=tk.VERTICAL, command=self.main_canvas.yview)
        self.scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # 3. Cấu hình Canvas
        self.main_canvas.configure(yscrollcommand=self.scrollbar.set)
        
        # 4. Frame chứa nội dung (nằm trong Canvas)
        self.content_frame = tk.Frame(self.main_canvas, bg=BG_COLOR)
        
        # --- [QUAN TRỌNG] Lưu ID của window để xử lý resize ---
        self.frame_id = self.main_canvas.create_window((0, 0), window=self.content_frame, anchor="nw")

        # Bind sự kiện để xử lý cuộn và giãn chiều ngang
        self.main_canvas.bind('<Configure>', self._on_canvas_configure)
        self.root.bind_all("<MouseWheel>", self._on_mousewheel)

        # --- BIẾN HỆ THỐNG ---
        self.robot = DifferentialDriveRobot()
        self.running = False
        self.mode_uart = False 
        self.curr_wl = 0.0
        self.curr_wr = 0.0
        self.ser = None
        self.serial_thread = None
        self.stop_thread = False

        # --- GIAO DIỆN ---
        self.setup_ui()
        self.reset_data()
        
        # Vẽ khởi tạo
        D, r = self.robot.params()
        draw_robot(self.ax_robot, 0, 0, 0, D, r, [], [])
        self.canvas_robot.draw()
        self.setup_initial_axes()

    def _on_canvas_configure(self, event):
        """Hàm này giúp nội dung luôn giãn đầy chiều ngang màn hình"""
        # Cập nhật vùng cuộn
        self.main_canvas.configure(scrollregion=self.main_canvas.bbox("all"))
        # Ép chiều rộng của frame nội dung bằng chiều rộng canvas
        self.main_canvas.itemconfig(self.frame_id, width=event.width)

    def _on_mousewheel(self, event):
        self.main_canvas.yview_scroll(int(-1*(event.delta/120)), "units")

    def setup_ui(self):

        left_col = tk.Frame(self.content_frame, bg=BG_COLOR, padx=10, pady=10)
        left_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # 1. Mô phỏng Robot
        tk.Label(left_col, text="MÔ PHỎNG", bg=BG_COLOR, font=("Arial", 10, "bold")).pack(anchor="w")
        fig_left = Figure(figsize=(4, 4), dpi=100, facecolor='white')
        fig_left.subplots_adjust(left=0.18, bottom=0.15)
        self.ax_robot = fig_left.add_subplot(111)
        self.canvas_robot = FigureCanvasTkAgg(fig_left, master=left_col)
        self.canvas_robot.get_tk_widget().pack()

        # 2. Bảng Điều khiển
    
        ctrl_group = tk.LabelFrame(left_col, text="BẢNG ĐIỀU KHIỂN", bg=PANEL_BG, padx=5, pady=5, font=("Arial", 9, "bold"), labelanchor='n')
        ctrl_group.pack(fill=tk.X, pady=10)

        # Thông số vật lý
        self.add_section_label(ctrl_group, "1. Thông số Vật lý:")
        phys_row = tk.Frame(ctrl_group, bg=PANEL_BG)
        phys_row.pack(pady=2) 
        self.R_entry = self.add_entry_compact(phys_row, "R(mm)", "120")
        self.D_entry = self.add_entry_compact(phys_row, "D(mm)", "250")
        self.M_entry = self.add_entry_compact(phys_row, "M(kg)", "1.2")

        # Nhập tay & UART Switch
        self.add_section_label(ctrl_group, "2. Chế độ điều khiển:")
        mode_row = tk.Frame(ctrl_group, bg=PANEL_BG)
        mode_row.pack(pady=2) 
        self.var_mode = tk.IntVar(value=0)
        tk.Radiobutton(mode_row, text="Manual", variable=self.var_mode, value=0, bg=PANEL_BG, command=self.toggle_mode).pack(side=tk.LEFT, padx=5)
        tk.Radiobutton(mode_row, text="UART", variable=self.var_mode, value=1, bg=PANEL_BG, command=self.toggle_mode).pack(side=tk.LEFT, padx=5)

        # Input Manual
        man_row = tk.Frame(ctrl_group, bg=PANEL_BG)
        man_row.pack(pady=5)
        self.wl_entry = self.add_entry_compact(man_row, "wL", "0.0")
        self.wr_entry = self.add_entry_compact(man_row, "wR", "0.0")
        tk.Button(man_row, text="GỬI", bg="#FF9800", fg="white", font=("Arial", 8, "bold"), width=6, command=self.update_manual_vel).pack(side=tk.LEFT, padx=5)

        # UART Connect
        uart_row = tk.Frame(ctrl_group, bg=PANEL_BG)
        uart_row.pack(pady=5) 
        self.cbo_port = ttk.Combobox(uart_row, width=8); self.cbo_port.pack(side=tk.LEFT)
        self.refresh_ports()
        self.entry_baud = tk.Entry(uart_row, width=6, justify="center"); self.entry_baud.insert(0, "9600"); self.entry_baud.pack(side=tk.LEFT, padx=2)
        self.btn_connect = tk.Button(uart_row, text="KẾT NỐI", bg="#607D8B", fg="white", font=("Arial", 8), command=self.toggle_uart)
        self.btn_connect.pack(side=tk.LEFT, padx=2)
        
       
        self.lbl_status = tk.Label(ctrl_group, text="Disconnected", fg="red", bg=PANEL_BG, font=("Arial", 8))
        self.lbl_status.pack(pady=(0,5))

        # Nút Sim
        sim_row = tk.Frame(ctrl_group, bg=PANEL_BG)
        sim_row.pack(pady=10) 
        tk.Button(sim_row, text="START", bg="green", fg="white", width=8, command=self.start_sim).pack(side=tk.LEFT, padx=5)
        tk.Button(sim_row, text="STOP", bg="red", fg="white", width=8, command=self.stop_sim).pack(side=tk.LEFT, padx=5)
        tk.Button(sim_row, text="RESET", bg="blue", fg="white", width=8, command=self.reset_all).pack(side=tk.LEFT, padx=5)

        # Monitor (Hiển thị số)
        mon_group = tk.LabelFrame(left_col, text="THÔNG SỐ (Realtime)", bg=PANEL_BG, padx=5, pady=5, font=("Arial", 9, "bold"))
        mon_group.pack(fill=tk.X, pady=5)
        
        self.lbl_X = self.add_monitor_row(mon_group, "X (m):")
        self.lbl_Y = self.add_monitor_row(mon_group, "Y (m):")
        self.lbl_Theta = self.add_monitor_row(mon_group, "Góc (rad):")
        self.lbl_V = self.add_monitor_row(mon_group, "Vận tốc (m/s):")
        self.lbl_Input = self.add_monitor_row(mon_group, "Input (wL, wR):", fg="blue")


        # === CỘT PHẢI (Chiếm toàn bộ phần còn lại) ===
        # Sử dụng expand=True để nó tự giãn ra lấp đầy khoảng trắng
        right_col = tk.Frame(self.content_frame, bg=BG_COLOR, padx=10, pady=10)
        right_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        tk.Label(right_col, text="ĐỒ THỊ", bg=BG_COLOR, font=("Arial", 10, "bold")).pack()

        # Tạo 4 đồ thị - Tự động giãn theo kích thước khung chứa
        fig_right = Figure(figsize=(4.5, 8), dpi=100, facecolor='white')
        self.ax_pos   = fig_right.add_subplot(411)
        self.ax_vel   = fig_right.add_subplot(412)
        self.ax_angle = fig_right.add_subplot(413)
        self.ax_wheel = fig_right.add_subplot(414)
        
        fig_right.subplots_adjust(hspace=0.6, left=0.1, right=0.95, top=0.96, bottom=0.05)
        
        self.canvas_plot = FigureCanvasTkAgg(fig_right, master=right_col)
        self.canvas_plot.get_tk_widget().pack(fill=tk.BOTH, expand=True)


    # --- HELPER FUNCTIONS ---
    def add_section_label(self, parent, text):
        tk.Label(parent, text=text, bg=PANEL_BG, font=("Arial", 8, "italic"), fg="#555").pack(pady=(5,0))

    def add_entry_compact(self, parent, label, default):
        frm = tk.Frame(parent, bg=PANEL_BG)
        frm.pack(side=tk.LEFT, padx=5) # Tăng padx từ 2 lên 5 cho thoáng
        tk.Label(frm, text=label, bg=PANEL_BG, font=("Arial", 8)).pack(side=tk.LEFT)
        e = tk.Entry(frm, width=6, justify="center")
        e.insert(0, default)
        e.pack(side=tk.LEFT)
        return e

    def add_monitor_row(self, parent, label, fg="black"):
        frm = tk.Frame(parent, bg="white", bd=1, relief=tk.SOLID)
        frm.pack(fill=tk.X, pady=1)
        tk.Label(frm, text=label, bg="white", width=12, anchor="w", font=("Arial", 8)).pack(side=tk.LEFT, padx=5)
        lbl = tk.Label(frm, text="0.00", bg="white", fg=fg, font=("Arial", 9, "bold"))
        lbl.pack(side=tk.RIGHT, padx=5)
        return lbl

    def refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.cbo_port['values'] = ports
        if ports: self.cbo_port.current(0)

    # --- LOGIC ---
    def setup_initial_axes(self):
# Cấu hình chung cho các trục
        for ax in [self.ax_pos, self.ax_vel, self.ax_angle, self.ax_wheel]:
            ax.set_facecolor("#f9f9f9")
            ax.grid(True, linestyle=':', alpha=0.6)
        
        self.ax_pos.set_ylabel("X, Y (m)")
        self.ax_vel.set_ylabel("v (m/s)")
        self.ax_angle.set_ylabel("Rad | Rad/s")
        self.ax_wheel.set_ylabel("wL, wR (rad/s)")

        # --- TẠO CÁC ĐỐI TƯỢNG LINE (Lưu vào biến để dùng lại) ---
        # Đồ thị 1: Vị trí X, Y
        self.line_x, = self.ax_pos.plot([], [], 'b', label="X")
        self.line_y, = self.ax_pos.plot([], [], 'g', label="Y")
        self.ax_pos.legend(fontsize='x-small', loc='upper left')

        # Đồ thị 2: Vận tốc dài
        self.line_v, = self.ax_vel.plot([], [], 'r')
        
        # Đồ thị 3: Góc & Vận tốc góc
        self.line_theta, = self.ax_angle.plot([], [], 'purple', label="θ")
        self.line_w, = self.ax_angle.plot([], [], 'orange', label="ω")
        self.ax_angle.legend(fontsize='x-small', loc='upper left')

        # Đồ thị 4: Vận tốc bánh xe
        self.line_wl, = self.ax_wheel.plot([], [], 'brown', label="L")
        self.line_wr, = self.ax_wheel.plot([], [], 'black', label="R")
        self.ax_wheel.legend(fontsize='x-small', loc='upper left')
        
        self.canvas_plot.draw()

    def reset_data(self):
        self.X, self.Y, self.V, self.W, self.Theta = [], [], [], [], []
        self.WL, self.WR = [], []

    def reset_all(self):
        self.running = False
        self.reset_data()
        self.robot = DifferentialDriveRobot()
        self.curr_wl = 0.0; self.curr_wr = 0.0
        
        D, r = self.robot.params()
        draw_robot(self.ax_robot, 0, 0, 0, D, r, [], [])
        self.canvas_robot.draw()
        
        self.ax_pos.clear(); self.ax_vel.clear(); self.ax_angle.clear(); self.ax_wheel.clear()
        self.setup_initial_axes()
        self.update_monitor_labels(0,0,0,0,0)

    def update_manual_vel(self):
        if self.var_mode.get() == 0:
            try:
                self.curr_wl = float(self.wl_entry.get())
                self.curr_wr = float(self.wr_entry.get())
            except: pass

    def toggle_mode(self):
        if self.var_mode.get() == 1 and not self.ser:
            messagebox.showwarning("Cảnh báo", "Vui lòng kết nối UART trước!")
            self.var_mode.set(0)

    # --- UART THREAD ---
    def toggle_uart(self):
        if not self.ser:
            try:
                port = self.cbo_port.get()
                baud = int(self.entry_baud.get())
                self.ser = serial.Serial(port, baud, timeout=1)
                self.stop_thread = False
                self.serial_thread = threading.Thread(target=self.uart_read_loop)
                self.serial_thread.daemon = True
                self.serial_thread.start()
                
                self.lbl_status.config(text=f"Connected", fg="green")
                self.btn_connect.config(text="NGẮT", bg="#f44336")
                self.var_mode.set(1) 
            except Exception as e:
                messagebox.showerror("Lỗi", str(e))
        else:
            self.stop_thread = True
            if self.ser: self.ser.close()
            self.ser = None
            self.lbl_status.config(text="Disconnected", fg="red")
            self.btn_connect.config(text="KẾT NỐI", bg="#607D8B")

    def uart_read_loop(self):
        while not self.stop_thread and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line and self.var_mode.get() == 1:
                    parts = line.split(',')
                    if len(parts) >= 2:
                        self.curr_wl = float(parts[0])
                        self.curr_wr = float(parts[1])
            except: pass
            time.sleep(0.01)

    # --- SIMULATION LOOP ---
    def start_sim(self):
        if self.running: return
        try:
            R = float(self.R_entry.get())/1000
            D = float(self.D_entry.get())/1000
            M = float(self.M_entry.get())
            cx, cy, cth = self.robot.x, self.robot.y, self.robot.theta
            self.robot = DifferentialDriveRobot(R, D, M)
            self.robot.x, self.robot.y, self.robot.theta = cx, cy, cth
        except: pass
        
        self.running = True
        self.loop()

    def stop_sim(self):
        self.running = False

    def loop(self):
        if not self.running: return
        
        dt = 0.05
        x, y, theta = self.robot.update(self.curr_wl, self.curr_wr, dt)
        
        self.X.append(x); self.Y.append(y)
        self.V.append(self.robot.v); self.W.append(self.robot.w)
        self.Theta.append(theta)
        self.WL.append(self.curr_wl); self.WR.append(self.curr_wr)
        
        D, r = self.robot.params()
        
        # --- TỐI ƯU 1: Chỉ truyền 200 điểm quỹ đạo gần nhất vào hàm vẽ robot ---
        # Giúp robot không bị lag khi chạy lâu (vì vết xanh lá cây quá dài)
        limit_path = 200
        draw_robot(self.ax_robot, x, y, theta, D, r, self.X[-limit_path:], self.Y[-limit_path:])
        self.canvas_robot.draw()
        
        self.update_monitor_labels(x, y, theta, self.robot.v, self.robot.w)
        
        # Cập nhật đồ thị mỗi 2 vòng lặp (100ms) để nhẹ gánh cho CPU
        if len(self.X) % 2 == 0:
            self.update_graphs()
            
        # --- TỐI ƯU 2: Giảm thời gian chờ xuống 30ms (khoảng 33 FPS) cho mượt ---
        self.root.after(20, self.loop)

    def update_monitor_labels(self, x, y, th, v, w):
        self.lbl_X.config(text=f"{x:.2f}")
        self.lbl_Y.config(text=f"{y:.2f}")
        self.lbl_Theta.config(text=f"{th:.2f}")
        self.lbl_V.config(text=f"{abs(v):.2f}")
        self.lbl_Input.config(text=f"{self.curr_wl:.1f} | {self.curr_wr:.1f}")

    def update_graphs(self):
        if not self.X: return

        # --- TỐI ƯU 3: Cắt dữ liệu (Sliding Window) ---
        # Chỉ lấy 200 mẫu cuối cùng để vẽ
        limit = 200
        
        # Lấy trục thời gian tương ứng (index)
        t_all = range(len(self.X))
        t_view = list(t_all)[-limit:]
        
        # Cập nhật dữ liệu mới vào các đường Line đã tạo (Thay vì xóa đi vẽ lại)
        self.line_x.set_data(t_view, self.X[-limit:])
        self.line_y.set_data(t_view, self.Y[-limit:])
        
        self.line_v.set_data(t_view, np.abs(self.V[-limit:]))
        
        self.line_theta.set_data(t_view, self.Theta[-limit:])
        self.line_w.set_data(t_view, self.W[-limit:])
        
        self.line_wl.set_data(t_view, self.WL[-limit:])
        self.line_wr.set_data(t_view, self.WR[-limit:])

        # --- Tự động co giãn trục (Rescale) ---
        # Vì ta dùng set_data nên trục không tự giãn, phải gọi hàm này
        for ax in [self.ax_pos, self.ax_vel, self.ax_angle, self.ax_wheel]:
            ax.relim()           # Tính toán lại giới hạn dữ liệu
            ax.autoscale_view()  # Cập nhật khung nhìn trục

        self.canvas_plot.draw()

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotGUI(root)
    root.mainloop()