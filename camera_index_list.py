import cv2

def list_available_cameras(max_index=10):
    print("Available camera indexes:")
    for index in range(max_index):
        cap = cv2.VideoCapture(index)
        if cap.read()[0]:
            print(f"Camera found at index {index}")
            cap.release()
        else:
            print(f"Index {index} not available")

list_available_cameras()
