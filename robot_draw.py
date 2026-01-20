import numpy as np
import matplotlib.pyplot as plt

def draw_robot(ax, x, y, theta, D, r, path_x=None, path_y=None):

    ax.clear()

    # --- 1. Vẽ Mốc (0,0) & Quỹ đạo ---
    ax.plot(0, 0, 'rx', markersize=8, label="Start") 
    ax.grid(True, linestyle=':', alpha=0.6)
    
    if path_x is not None and len(path_x) > 1:
        ax.plot(path_x, path_y, color='green', linestyle='--', linewidth=1)

    # --- 2. Chuẩn bị thông số vẽ ---
    body_size = D 
    
    # Bánh xe
    wheel_len = r   # Đường kính bánh xe vẽ 
    wheel_wid = r * 0.5  # Bề rộng bánh xe
    
    # Khoảng cách từ tâm xe đến tâm bánh xe (theo trục Y local)
    wheel_dist_y = body_size / 2 + wheel_wid / 2 + 0.02

    # Ma trận xoay R
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s], [s, c]])

    def transform(points):
        # Hàm hỗ trợ xoay và dịch chuyển điểm về toạ độ toàn cục
        return (R @ points.T).T + np.array([x, y])

    # --- 3. VẼ CÁC CHI TIẾT ---

    # A. KHỚP NỐI (Joints) - Vẽ trước để nó nằm dưới các layer khác
    # Nối từ tâm thân xe ra tâm bánh xe
    joint_width = 0.02
    joint_len = wheel_dist_y * 2
    joint = np.array([
        [-joint_width, -joint_len/2], [joint_width, -joint_len/2],
        [joint_width, joint_len/2], [-joint_width, joint_len/2]
    ])
    ax.fill(*transform(joint).T, color="#555555") # Màu xám đậm

    # B. THÂN XE (Hình vuông D x D)
    body = np.array([
        [-body_size/2, -body_size/2],
        [ body_size/2, -body_size/2],
        [ body_size/2,  body_size/2],
        [-body_size/2,  body_size/2],
        [-body_size/2, -body_size/2] # Khép kín
    ])
    ax.fill(*transform(body).T, color="#FFC107", edgecolor="black", linewidth=2) # Màu vàng nghệ

    # C. CỤC HÌNH VUÔNG NHỎ (Block) - Trên thân xe, chỗ nối ra bánh
    block_size = body_size * 0.2 # Kích thước cục nhỏ tỉ lệ theo thân
    for sign in [-1, 1]: # Vẽ bên trái (-1) và phải (1)
        # Tọa độ cục nhỏ (nằm sát mép hông)
        bx = 0
        by = sign * (body_size/2 - block_size/2)
        
        block = np.array([
            [bx - block_size/2, by - block_size/2],
            [bx + block_size/2, by - block_size/2],
            [bx + block_size/2, by + block_size/2],
            [bx - block_size/2, by + block_size/2]
        ])
        ax.fill(*transform(block).T, color="#38FF22", edgecolor="black") # Màu cam đậm

    # D. BÁNH XE (Màu trắng, sọc đen)
    # Hàm vẽ 1 bánh xe
    def draw_wheel(center_y):
        # Hình chữ nhật bánh xe
        w_poly = np.array([
            [-wheel_len/2, center_y - wheel_wid/2],
            [ wheel_len/2, center_y - wheel_wid/2],
            [ wheel_len/2, center_y + wheel_wid/2],
            [-wheel_len/2, center_y + wheel_wid/2]
        ])
        trans_poly = transform(w_poly)
        ax.fill(trans_poly[:,0], trans_poly[:,1], color="white", edgecolor="black", linewidth=1.5)
        
        # Vẽ sọc đen (Stripes) trên bánh xe
        num_stripes = 4
        for i in range(num_stripes):
            # Tọa độ x dọc theo bánh xe
            lx = -wheel_len/2 + (i+1) * (wheel_len / (num_stripes + 1))
            
            # Tạo đường kẻ ngang
            stripe = np.array([
                [lx, center_y - wheel_wid/2],
                [lx, center_y + wheel_wid/2]
            ])
            trans_stripe = transform(stripe)
            ax.plot(trans_stripe[:,0], trans_stripe[:,1], color="black", linewidth=1.5)

    # Vẽ bánh trái và phải
    draw_wheel(-wheel_dist_y) # Bánh phải
    draw_wheel(wheel_dist_y)  # Bánh trái

    # E. MŨI TÊN HƯỚNG (Màu đỏ)
    heading_start = np.array([0, 0])
    heading_end = np.array([body_size/2 + 0.2, 0]) # Mũi tên dài hơn thân xe chút
    
    p_start = transform(np.array([heading_start]))[0]
    p_end = transform(np.array([heading_end]))[0]
    
    ax.arrow(p_start[0], p_start[1], 
             p_end[0]-p_start[0], p_end[1]-p_start[1],
             head_width=0.08, head_length=0.1, fc='red', ec='red', zorder=10)

    # --- 4. Cấu hình Camera ---
    ax.set_aspect("equal")
    
    view_range = max(D, 0.5) * 3 
    ax.set_xlim(x - view_range/2, x + view_range/2)
    ax.set_ylim(y - view_range/2, y + view_range/2)
    
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
