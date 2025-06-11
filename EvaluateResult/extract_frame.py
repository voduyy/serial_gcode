import glob
import cv2
import numpy as np
from PIL import Image
import os
from EvaluateResult import helper_image
from PIL import ImageEnhance, ImageFilter
# === Hàm resize giữ nguyên tỉ lệ, thêm padding trắng ===
def resize_with_padding(image, target_size=(320, 240), fill_color=(255, 255, 255)):
    original_ratio = image.width / image.height # Tính tỉ lệ gốc của ảnh
    target_ratio = target_size[0] / target_size[1]  # Tỉ lệ mong muốn (320x240)

    if original_ratio > target_ratio:
        new_width = target_size[0] # Resize theo chiều ngang
        new_height = round(new_width / original_ratio)
    else:
        new_height = target_size[1]
        new_width = round(new_height * original_ratio)  # Resize theo chiều dọc

    resized_image = image.resize((new_width, new_height), Image.Resampling.LANCZOS)
    new_image = Image.new("RGB", target_size, fill_color) # Tạo ảnh nền trắng 320x240
    offset = ((target_size[0] - new_width) // 2, (target_size[1] - new_height) // 2) # Căn giữa ảnh nhỏ lên nền
    new_image.paste(resized_image, offset) # Ghép vào giữa
    return new_image

def preprocessing(img_name):
# # === Xử lý từng ảnh ===
#     base_dir = os.path.dirname(os.path.abspath(__file__))
#     img_path = os.path.join(base_dir,img_path)
    base_dir = os.path.dirname(os.path.abspath(__file__))
    full_path = os.path.join(base_dir, "input_capture_excam", img_name)

    img = cv2.imread(full_path)
    # img = cv2.flip(img, 1)
    if img is None:
        return None
    # === Thư mục đầu ra ===
    output_frame_dir = 'output_frame_w_image' # đầu ra cắt mỗi khung vẫn giữ lại viền
    output_inside_dir = 'output_image' # đầu ra chỉ còn ảnh bên trong khung
    os.makedirs(output_frame_dir, exist_ok=True)
    os.makedirs(output_inside_dir, exist_ok=True)
    filtered = helper_image.filter_color(img)
    img = cv2.cvtColor(filtered, cv2.COLOR_GRAY2BGR)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY_INV)  # Nhị phân hóa để dễ tìm khung
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # Tìm contour ngoài cùng

    max_area = 0
    frame_contour = None
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)  # Làm mượt contour
        area = cv2.contourArea(cnt)
        if len(approx) == 4 and area > max_area:  # Chỉ chọn contour 4 góc có diện tích lớn nhất
            max_area = area
            frame_contour = approx

    if frame_contour is not None:
        x, y, w, h = cv2.boundingRect(frame_contour)  # Lấy tọa độ khung chữ nhật

        # # === output_frame: CÓ KHUNG, không resize ===
        frame_crop = img[y:y + h, x:x + w]  # Cắt nguyên khung
        frame_filename = img_name
        frame_path = os.path.join(output_frame_dir, frame_filename)
        cv2.imwrite(frame_path, frame_crop)

        # === output_insideFrame: KHÔNG KHUNG, có resize giữ tỉ lệ ===
        margin = 5  # loại bỏ viền khung
        inner_crop = img[y + margin:y + h - margin, x + margin:x + w - margin]
        pil_image = Image.fromarray(cv2.cvtColor(inner_crop, cv2.COLOR_BGR2RGB))  # Chuyển sang RGB
        resized = resize_with_padding(pil_image, target_size=(320, 240))
        resized = resized.convert("RGB")

        # Làm sắc nét ảnh bằng PIL
        resized = resized.filter(ImageFilter.UnsharpMask(radius=2, percent=150, threshold=3))

        inside_path = os.path.join(base_dir, output_inside_dir, frame_filename)
        resized.save(inside_path, dpi=(600, 600), quality=100, progressive=True)

        print(f"✅ OK: {os.path.basename(full_path)} → xử lý thành công")
        return None
    else:
        print(f"❌ Không tìm thấy khung trong ảnh: {os.path.basename(full_path)}")
        return None
