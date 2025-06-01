import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
import re
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import global_var
# ===== Định nghĩa robot =====
# L1, L2, L3, L4 = 0.1537, 0.1433, 0.077, 0.1203
# ===== Hàm tính G-code giữ G0/G1 và ưu tiên nghiệm gần nhất =====
L1, L2, L3, L4 = 0.1537, 0.1433, 0.0, 0.1443
laser_extension = 0.075  # 7.5 cm
deg = np.pi / 180
robot = DHRobot([
    RevoluteDH(d=L1, a=0, alpha=-np.pi / 2, offset=np.pi / 2, qlim=[-90 * deg, 90 * deg]),
    RevoluteDH(d=0, a=L2, alpha=0, offset=-np.pi / 2, qlim=[-90 * deg, 90 * deg]),
    RevoluteDH(d=L3, a=0, alpha=np.pi / 2, offset=np.pi / 2, qlim=[-120 * deg, 120 * deg]),
    RevoluteDH(d=L4, a=0, alpha=np.pi / 2, offset=0, qlim=[-180 * deg, 180 * deg])

], name='3DOF_Robot')
q = np.zeros(4)  # Khởi tạo nghiệm ban đầu là 0
# robot.teach(q)
# ===== Hệ số chuyển đổi góc → bước (mm) =====
STEP_CONVERT = {
    'X': 0.355555,
    'Y': 0.355555,
    'Z': 0.216666,
    'A': 0.133333,
}
def compute_gcode_line(cmd, x, y, z, q0=None, max_attempts=10):
    # Mục tiêu: Y luôn là [0, -1, 0]
    y_axis = np.array([0, -1, 0])  # Hướng Y của đầu cuối
    # Giả sử trục Z hướng lên (hoặc trùng với hướng làm việc)
    z_axis = np.array([0, 0, 0])
    # Dùng tích có hướng để tìm trục X vuông góc với Y và Z
    x_axis = np.cross(y_axis, z_axis)
    # Xây ma trận quay (mỗi cột là một trục)
    R_goal = np.column_stack((x_axis, y_axis, z_axis))
    # Vị trí mong muốn
    # T_goal = SE3(R_goal) * SE3(x, y, z)
    T_goal = SE3(x, y, z)
    for attempt in range(max_attempts):
        ik_result = robot.ikine_LM(T_goal, q0=q0, mask=[1, 1, 1, 0, 1, 0])
        if ik_result.success:
            T_actual = robot.fkine(ik_result.q)
            # y_actual = T_actual.R[:, 1]
            # print("🔍 Y hướng thực tế của đầu cuối:", y_actual)
        if not ik_result.success:
            continue

        q_deg = np.degrees(ik_result.q)

        if -90 < q_deg[0] < 90 and -120 < q_deg[1] < 120 and -120 < q_deg[2] < 120:
            x_step = -q_deg[0] * STEP_CONVERT['X']
            y_step = -q_deg[1] * STEP_CONVERT['Y']
            z_step = q_deg[2] * STEP_CONVERT['Z']
            a_step = q_deg[3] * STEP_CONVERT['A']
            gcode_line = f"{cmd} X{x_step:.3f} Y{y_step:.3f} Z{z_step:.3f} A{a_step:.3f} F2700"
            return gcode_line, q_deg, ik_result.q

    return None, None, None
def main():

    # ===== Đọc và xử lý file G-code =====
    # Lấy đường dẫn thư mục chứa file script đang chạy
    base_dir = os.path.dirname(os.path.abspath(__file__))

    # Đi lên một cấp thư mục
    parent_dir = os.path.abspath(os.path.join(base_dir, ".."))
    path_input_file = os.path.join(parent_dir, 'Image2Gcode', 'output_gcode')
    input_file = os.path.join(path_input_file, f"{global_var.index_capture_image}_gcode.nc")
    filename= f"{global_var.index_capture_image}_gcode.nc"
    print(input_file)

    pattern = re.compile(
        r"^(G0|G1|G92)\s+.*?X([-+]?\d*\.?\d+)\s+Y([-+]?\d*\.?\d+)(?:\s+Z([-+]?\d*\.?\d+))?(?:\s+A([-+]?\d*\.?\d+))?",
        re.IGNORECASE
    )

    gcode_lines = []
    q_list = []
    q0 = None  # Khởi tạo nghiệm ban đầu là None

    def set_axes_equal(ax):
        import numpy as np

        x_limits = ax.get_xlim3d()
        y_limits = ax.get_ylim3d()
        z_limits = ax.get_zlim3d()

        x_range = abs(x_limits[1] - x_limits[0])
        y_range = abs(y_limits[1] - y_limits[0])
        z_range = abs(z_limits[1] - z_limits[0])

        max_range = max([x_range, y_range, z_range])

        x_middle = np.mean(x_limits)
        y_middle = np.mean(y_limits)
        z_middle = np.mean(z_limits)

        ax.set_xlim3d([x_middle - max_range / 2, x_middle + max_range / 2])
        ax.set_ylim3d([y_middle - max_range / 2, y_middle + max_range / 2])
        ax.set_zlim3d([z_middle - max_range / 2, z_middle + max_range / 2])

    # === Giai đoạn 1: Đọc toàn bộ để tìm min/max G-code ===
    x_gcode_vals = []
    y_gcode_vals = []

    with open(input_file, "r") as f:
        gcode_raw_lines = f.readlines()
    for line in gcode_raw_lines:
        match = pattern.search(line.strip())
        if match:
            x_gcode_vals.append(float(match.group(2)))
            y_gcode_vals.append(float(match.group(3)))
    # === Tính tâm và kích thước gốc ===
    x_min, x_max = min(x_gcode_vals), max(x_gcode_vals)
    y_min, y_max = min(y_gcode_vals), max(y_gcode_vals)
    x_center = (x_min + x_max) / 2
    y_center = (y_min + y_max) / 2
    gcode_width = max(x_max - x_min, y_max - y_min)
    print(f"🧠 GCODE_WIDTH: {gcode_width:.2f} mm | Tâm gốc: ({x_center:.2f}, {y_center:.2f})")

    # === Giai đoạn 2: Biến đổi và sinh G-code ===
    DRAW_WIDTH = 0.1  # mét
    SCALE = DRAW_WIDTH / gcode_width
    # ===== Di chuyển đến điểm tâm ban đầu =====
    x_init, y_init, z_init = 0.0, 0.2, 0.15  # Điểm trung tâm
    init_line, q_deg_init, q_rad_init = compute_gcode_line("G1", x_init, y_init, z_init)
    if init_line:
        print(f"🚀 Di chuyển đến tâm: {init_line}")
        print("🔧 Góc khớp (deg): q1 = {:.2f}, q2 = {:.2f}, q3 = {:.2f}, q4 = {:.2f}".format(*q_deg_init))
        gcode_lines.append(init_line)
        q_list.append(q_deg_init)
        q0 = q_rad_init  # Cập nhật nghiệm gần nhất
    else:
        print("❌ Không thể di chuyển đến điểm tâm đầu (0, 0.2, 0.15)")
    # ===== Đọc lại G-code và xử lý từng dòng =====
    for line in gcode_raw_lines:
        line = line.strip()
        match = pattern.search(line)
        if not match:
            if line.startswith(("M3", "M5", "G28")):
                gcode_lines.append(line)
            continue

        cmd = match.group(1).upper()
        x_gcode = float(match.group(2))
        y_gcode = float(match.group(3))

        # Scale và dịch tâm
        x = (x_gcode - x_center) * SCALE
        z = (y_gcode - y_center) * SCALE + 0.15  # đặt tâm tại Z = 0.15
        y = 0.2  # chiều cao cố định

        gcode_line, q_deg, q_rad = compute_gcode_line(cmd, x, y, z, q0=q0)
        if q_rad is None:
            print(f"❌ IK thất bại tại điểm ({x:.3f}, {y:.3f}, {z:.3f})")
        else:
            print(f"✅ {gcode_line}")
            print("🔧 Góc khớp (deg): q1 = {:.2f}, q2 = {:.2f}, q3 = {:.2f}, q4 = {:.2f}".format(*q_deg))
            print(f"🔧 Thành công tại điểm: ({x:.3f}, {y:.3f}, {z:.3f})")
            gcode_lines.append(gcode_line)
            q_list.append(q_deg)
            q0 = q_rad

    # ===== Ghi file kết quả G-code =====
    output_file = f"final_gcode/face_gcode/{global_var.index_capture_image}.txt"
    print(output_file)
    with open(output_file, "w") as f:
        for line in gcode_lines:
            f.write(line + "\n")
    print(f"\n✅ Đã lưu {len(gcode_lines)} dòng vào '{output_file}'")

    # ===== Ghi file góc khớp ra file riêng =====
    # angle_file = f"RobotGcodeGen/output_degree/{global_var.index_capture_image}.txt"
    # with open(angle_file, "w") as f:
    #     for q_deg in q_list:
    #         f.write("{:.4f},{:.4f},{:.4f},{:.4f}\n".format(*q_deg))
    # print(f"✅ Đã lưu góc khớp vào '{angle_file}'")
#===== Plot đường đi đầu cuối =====
# if q_list:
#     positions = [robot.fkine(np.radians(q)).t for q in q_list]
#     xs = [p[0] for p in positions]  # X thật
#     ys = [p[1] for p in positions]  # Z thật gán cho Y plot
#     zs = [p[2] for p in positions]  # Y thật gán cho Z plot
#
#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')
#     ax.plot(xs, zs, ys, marker='o', label='Quỹ đạo đầu cuối')
#     ax.set_xlabel("X (m)")
#     ax.set_ylabel("Z (m)")  # đổi nhãn ở đây
#     ax.set_zlabel("Y (m)")  # đổi nhãn ở đây
#     # print("Giá trị ys (trục Z plot):", ys)
#     ax.set_title("Đường đi thực tế của đầu cuối robot")
#     ax.legend()
#     ax.view_init(elev=45, azim=45)
#     set_axes_equal(ax)
#     # ==== Vẽ khung hình vuông ảo (15x15cm) ====
#     square_size = 0.1  # mét
#     center_x = 0.0
#     center_z = 0.15
#     fixed_y = 0.2
#
#     half = square_size / 2
#     square_x = [center_x - half, center_x + half, center_x + half, center_x - half, center_x - half]
#     square_z = [center_z - half, center_z - half, center_z + half, center_z + half, center_z - half]
#     square_y = [fixed_y] * 5  # Giữ nguyên Y
#
#     ax.plot(square_x, square_z, square_y, color='r', linestyle='--', label='Khung vẽ 15x15cm')
#     plt.show()
#
# else:
#     print("⚠️ Không có điểm nào để vẽ quỹ đạo.")
if __name__ == "__main__":
    main()