import csv
import matplotlib.pyplot as plt

# Tên file CSV bạn vừa log
csv_filename = 'hihix.csv'

# Các mảng dữ liệu
time_data = []
freq_data = []
count_data = []

# Đọc dữ liệu từ CSV
with open(csv_filename, mode='r') as file:
    reader = csv.DictReader(file)
    for row in reader:
        time_data.append(float(row['Time (s)']))
        freq_data.append(float(row['Frequency (Hz)']))
        count_data.append(int(row['Count']))

# Vẽ đồ thị
plt.figure(figsize=(10, 6))

# Vẽ frequency theo thời gian
plt.subplot(2, 1, 1)
plt.plot(time_data, freq_data, label='Frequency (Hz)')
plt.xlabel('Time (s)')
plt.ylabel('Frequency (Hz)')
plt.title('Frequency vs Time')
plt.grid(True)
plt.legend()

# # Vẽ count theo thời gian
# plt.subplot(2, 1, 2)
# plt.plot(time_data, count_data, label='Count', color='orange')
# plt.xlabel('Time (s)')
# plt.ylabel('Count')
# plt.title('Count vs Time')
# plt.grid(True)
# plt.legend()

plt.tight_layout()
plt.show()
