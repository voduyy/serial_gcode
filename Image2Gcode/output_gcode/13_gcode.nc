import matplotlib.pyplot as plt
import math

def rotate_point(x, y, angle_deg, origin=(0, 0)):
    angle_rad = math.radians(angle_deg)
    ox, oy = origin
    tx, ty = x - ox, y - oy
    rx = tx * math.cos(angle_rad) - ty * math.sin(angle_rad)
    ry = tx * math.sin(angle_rad) + ty * math.cos(angle_rad)
    return rx + ox, ry + oy

def parse_gcode_line(line):
    x = y = None
    if line.startswith(('G0', 'G1')):
        parts = line.split()
        for part in parts:
            if part.startswith('X'):
                x = float(part[1:])
            elif part.startswith('Y'):
                y = float(part[1:])
    return x, y

def draw_gcode_and_save(file_path, output_image_path="output.png", angle_deg=0):
    with open(file_path, 'r') as file:
        lines = file.readlines()

    segments = []
    colors = []
    current_x, current_y = 0, 0

    for line in lines:
        line = line.strip()
        if line.startswith(('G0', 'G1')):
            x, y = parse_gcode_line(line)
            x = x if x is not None else current_x
            y = y if y is not None else current_y
            segments.append(((current_x, current_y), (x, y)))
            colors.append('red' if line.startswith('G0') else 'blue')
            current_x, current_y = x, y

    # Xoay quanh tâm
    all_x = [pt[0] for seg in segments for pt in seg]
    all_y = [pt[1] for seg in segments for pt in seg]
    center_x = (max(all_x) + min(all_x)) / 2
    center_y = (max(all_y) + min(all_y)) / 2

    fig, ax = plt.subplots(figsize=(8, 8))
    for ((x0, y0), (x1, y1)), color in zip(segments, colors):
        x0r, y0r = rotate_point(x0, y0, angle_deg, (center_x, center_y))
        x1r, y1r = rotate_point(x1, y1, angle_deg, (center_x, center_y))
        ax.plot([x0r, x1r], [y0r, y1r], color=color, alpha=0.4 if color == 'red' else 1.0)

    ax.invert_yaxis()
    ax.invert_xaxis()
    ax.set_aspect('equal')
    ax.axis('off')  # Không hiển thị trục
    plt.tight_layout()
    plt.savefig(output_image_path, dpi=300)
    plt.close()
