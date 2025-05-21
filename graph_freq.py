import serial
import csv
import time

# Cấu hình COM port
port = 'COM15'    # Đổi đúng COM bạn đang dùng
baudrate = 921600
timeout = 1

# Tạo đối tượng Serial
ser = serial.Serial(port, baudrate, timeout=timeout)

# File CSV để ghi
csv_filename = 'log_freq_x_finall1.csv'

with open(csv_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    # Ghi header
    writer.writerow(['Time (s)', 'Frequency (Hz)', 'Count'])

    start_time = time.time()

    try:
        while True:
            line = ser.readline().decode('utf-8').strip()
            if line:
                print("Received:", line)

                try:
                    # Tách bằng TAB '\t' thay vì dấu phẩy
                    freq_str, count_str = line.split('\t')
                    freq_value = float(freq_str)
                    count_value = int(count_str)
                    elapsed_time = time.time() - start_time

                    # Ghi vào CSV
                    writer.writerow([elapsed_time, freq_value, count_value])
                    file.flush()
                except (ValueError, IndexError):
                    print("⚠️ Parse error:", line)
                    pass

    except KeyboardInterrupt:
        print("Đã dừng ghi dữ liệu.")
        ser.close()
