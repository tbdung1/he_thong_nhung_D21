import serial
import time

# Cấu hình cổng COM mà ESP32 đang sử dụng
# Lưu ý thay 'COMx' bằng cổng COM thực tế của ESP32 (thường là COM3, COM4,... trên Windows)
ser = serial.Serial('COM7', 115200)  # 115200 là baud rate, phải khớp với baud rate của ESP32
time.sleep(2)  # Đợi ESP32 khởi động

while True:
    if ser.in_waiting > 0:
        data = ser.readline().decode('utf-8').strip()  # Đọc dữ liệu từ ESP32
        print(f"Dữ liệu từ ESP32: {data}")

        # Tại đây bạn có thể phân tích và đưa dữ liệu vào mô hình dự đoán
        # Chẳng hạn, chia nhỏ dữ liệu để sử dụng (ví dụ: độ ẩm đất, nhiệt độ)
        # Bạn có thể thực hiện các phép toán hoặc chuyển dữ liệu vào mô hình ML

    time.sleep(1)  # Đọc lại sau 1 giây
