import csv
import random
import time
from datetime import datetime, timedelta

# Đường dẫn file CSV
file_path = "C://Users//84946//OneDrive//Desktop//Documents PTIT//HK2_nam4//Lap_trinh_nhung//project//Data//soil_moisture_data.csv"

# Số lượng mẫu dữ liệu giả lập
num_samples = 500

def generate_fake_data(base_time):
    """Sinh dữ liệu giả lập với giá trị nhiệt độ, độ ẩm và độ ẩm đất hợp lý."""
    temperature = round(random.uniform(20, 35), 2)  # Nhiệt độ từ 20°C đến 35°C
    humidity = round(random.uniform(30, 80), 2)  # Độ ẩm không khí từ 30% đến 80%
    soil_moisture = random.randint(500, 3000)  # Độ ẩm đất trong khoảng 500 - 3000 (tùy chỉnh theo cảm biến)
    time_offset = timedelta(seconds=random.randint(30, 600))  # Ngẫu nhiên từ 30 giây đến 10 phút
    timestamp = (base_time + time_offset).strftime("%Y-%m-%d %H:%M:%S")
    return [timestamp, soil_moisture, temperature, humidity]

# Ghi dữ liệu giả lập vào file CSV
with open(file_path, mode='a', newline='') as file:
    writer = csv.writer(file)
    
    # Nếu file trống, ghi tiêu đề
    if file.tell() == 0:
        writer.writerow(["Time", "SoilMoisture", "Temperature", "Humidity"])
    
    base_time = datetime.now()
    for _ in range(num_samples):
        fake_data = generate_fake_data(base_time)
        writer.writerow(fake_data)
        base_time += timedelta(seconds=random.randint(10, 900))  # Cập nhật thời gian cho lần tiếp theo

print(f"Đã tạo {num_samples} mẫu dữ liệu giả lập và lưu vào {file_path}")
