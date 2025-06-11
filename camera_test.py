import cv2

def nothing(x):
    pass

# Mở camera
cap = cv2.VideoCapture(1)  # Thay 0 bằng 1 nếu dùng camera ngoài

# Tạo cửa sổ và các thanh trượt
cv2.namedWindow('Camera Settings')

cv2.createTrackbar('Brightness', 'Camera Settings', 50, 255, nothing)
cv2.createTrackbar('Contrast', 'Camera Settings', 50, 255, nothing)
cv2.createTrackbar('Saturation', 'Camera Settings', 50, 255, nothing)

while True:
    # Đọc giá trị từ thanh trượt
    brightness = cv2.getTrackbarPos('Brightness', 'Camera Settings')
    contrast = cv2.getTrackbarPos('Contrast', 'Camera Settings')
    saturation = cv2.getTrackbarPos('Saturation', 'Camera Settings')

    # Cập nhật các thông số camera
    cap.set(cv2.CAP_PROP_BRIGHTNESS, brightness)
    cap.set(cv2.CAP_PROP_CONTRAST, contrast)
    cap.set(cv2.CAP_PROP_SATURATION, saturation)

    ret, frame = cap.read()
    if not ret:
        break

    cv2.imshow('Camera Preview', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
