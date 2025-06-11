import cv2
import numpy as np
from matplotlib import pyplot as plt

# Load image in color
def filter_color(image):
    # Define black color range (BGR)
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([100, 100, 100])  # Cho phép một chút gần đen

    # Create mask: giữ pixel nằm trong vùng "đen"
    mask_black = cv2.inRange(image, lower_black, upper_black) # pixel trong khoảng thành màu trắng, ngoài đen

    # Invert to get binary: black stays black (0), others become white (255)
    binary_image = cv2.bitwise_not(mask_black)
    return binary_image