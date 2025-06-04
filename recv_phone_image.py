import socket
import os
import global_var
# Cấu hình server
SERVER_IP = "0.0.0.0"  # Lắng nghe trên mọi địa chỉ IP
SERVER_PORT = 8988
BUFFER_SIZE = 4096  # Kích thước buffer để đọc dữ liệu

# Đảm bảo thư mục để lưu tệp đã được tạo
SAVE_DIR = "phone_image"
if not os.path.exists(SAVE_DIR):
    os.makedirs(SAVE_DIR)

def receive_file():
    # Tạo một socket TCP
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.bind((SERVER_IP, SERVER_PORT))  # Liên kết IP và cổng
        server_socket.listen(1)  # Lắng nghe 1 kết nối tại một thời điểm

        print(f"Server đang lắng nghe tại {SERVER_IP}:{SERVER_PORT}")
        conn, addr = server_socket.accept()  # Chấp nhận kết nối
        with conn:
            print(f"Đã kết nối với {addr}")
            # Nhận tên tệp từ client
            file_name_length = int.from_bytes(conn.recv(4), byteorder='big')  # Nhận chiều dài của tên tệp
            file_name = conn.recv(file_name_length).decode("utf-8")  # Nhận tên tệp với chiều dài đã biết
            global_var.index_capture_image +=1
            file_name = f"{global_var.index_capture_image}.jpg"
            global_var.image_name = file_name
            print(f"Nhận tệp: {file_name}")

            # Xác định đường dẫn lưu tệp
            save_path = os.path.join(SAVE_DIR, file_name)

            # Nhận dữ liệu tệp và ghi vào file
            with open(save_path, "wb") as file:
                while True:
                    data = conn.recv(BUFFER_SIZE)
                    if not data:
                        break
                    file.write(data)
                    print(f"Đã nhận {len(data)} byte dữ liệu")

            print(f"Tệp {file_name} đã nhận và lưu thành công tại {save_path}")
    return save_path
# def close_socket():
#     server_socket.close()
# if __name__ == "__main__":
#     receive_file()
