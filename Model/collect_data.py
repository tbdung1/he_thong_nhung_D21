import serial
import csv
import time
import os

# Cấu hình cổng COM của ESP32
ser = serial.Serial('COM7', 115200)  # Đảm bảo thay COMx bằng cổng thực tế của bạn
time.sleep(2)

# Kiểm tra xem file đã tồn tại chưa
file_path = 'C://Users//84946//OneDrive//Desktop//Documents PTIT//HK2_nam4//Lap_trinh_nhung//project//Data//soil_moisture_data.csv'

# Mở file CSV để lưu dữ liệu (nếu file không tồn tại thì tạo mới, nếu có rồi thì không ghi tiêu đề)
with open(file_path, mode='a', newline='') as file:
    writer = csv.writer(file)

    # Nếu file mới, ghi tiêu đề cột
    if os.stat(file_path).st_size == 0:
        writer.writerow(['Time', 'SoilMoisture', 'Temperature', 'Humidity'])  # Tiêu đề các cột

    while True:
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8').strip()
            print(f"Dữ liệu từ ESP32: '{data}'")  # In ra dữ liệu, bao quanh bằng dấu nháy đơn để kiểm tra

            if "Bật bơm nước" in data or "Tắt bơm nước" in data:
                # print("Thông báo điều khiển bơm, bỏ qua")
                continue

            # Loại bỏ ký tự tab \t, thay bằng khoảng trắng (hoặc dấu phẩy nếu cần)
            data = data.replace('\t', ',')  # Hoặc dùng data.split("\t") nếu bạn muốn giữ riêng biệt các phần

            # Kiểm tra xem dữ liệu có chứa "Temperature:", "Humidity:", và "SoilMoisture:" hay không
            if "Temperature:" in data and "Humidity:" in data and "SoilMoisture:" in data:
                try:
                    # Tách dữ liệu theo dấu cách
                    parts = data.split(',')

                    # Kiểm tra nếu dữ liệu có đúng 3 phần (Temperature, Humidity, SoilMoisture)
                    if len(parts) == 3:
                        # Chia các phần theo ":"
                        temperature = float(parts[0].split(":")[1].strip())  # Phần nhiệt độ
                        humidity = float(parts[1].split(":")[1].strip())  # Phần độ ẩm không khí
                        soil_moisture = int(parts[2].split(":")[1].strip())  # Phần độ ẩm đất

                        # Lưu dữ liệu vào CSV
                        writer.writerow([time.strftime("%Y-%m-%d %H:%M:%S"), soil_moisture, temperature, humidity])
                    else:
                        print("Dữ liệu không đầy đủ, bỏ qua")
                except ValueError as e:
                    print(f"Lỗi phân tích dữ liệu: {e}")
            else:
                print("Không phải dữ liệu cảm biến, bỏ qua")

        time.sleep(1)  # Đọc lại sau 1 giây
