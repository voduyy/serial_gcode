import serial
import pygame
import time


# Setup pygame
pygame.init()
screen = pygame.display.set_mode((800, 600))
pygame.display.set_caption("G-code Visualizer")
clock = pygame.time.Clock()

# Biến trạng thái
current_pos = [0, 0]
scale = 2  # Phóng to hình ảnh
offset = [400, 300]  # Tâm hình


def draw_line(start, end, color=(0, 0, 255)):
    pygame.draw.line(screen, color,
                     (start[0] * scale + offset[0], start[1] * scale + offset[1]),
                     (end[0] * scale + offset[0], end[1] * scale + offset[1]))

def parse_and_execute_gcode_line(line):
    global current_pos
    line = line.strip()
    if line.startswith(("G0", "G1")):
        x = y = None
        parts = line.split()
        for part in parts:
            if part.startswith("X"):
                x = float(part[1:])
            elif part.startswith("Y"):
                y = float(part[1:])
        x = x if x is not None else current_pos[0]
        y = y if y is not None else current_pos[1]
        draw_line(current_pos, (x, y), color=(200, 0, 0) if line.startswith("G0") else (0, 255, 0))
        current_pos = [x, y]

def run_gcode_visualizer(gcode_path):
    with open(gcode_path) as f:
        lines = f.readlines()

    for line in lines:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return
        parse_and_execute_gcode_line(line)
        pygame.display.flip()
        clock.tick(30)

    while True:  # Giữ cửa sổ sau khi chạy xong
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return

# Chạy

screen.fill((0, 0, 0))
run_gcode_visualizer(f"output_gcode/{global_var.index_gcode_command}_gcode.nc")
