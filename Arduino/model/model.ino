#include <Arduino.h>
#include <DHT.h>
#include <TensorFlowLite_ESP32.h>

#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"

// 📌 Load model đã convert
#include "nhung_model.h"  

// 📌 Cấu hình cảm biến
#define DHTPIN 19          // Chân nối DHT11
#define DHTTYPE DHT11      // Loại cảm biến
#define SOIL_MOISTURE_PIN 32  // Cảm biến độ ẩm đất
#define RELAY_PIN 23   // Chân điều khiển relay

DHT dht(DHTPIN, DHTTYPE);  

// 📌 Khai báo TensorFlow Lite
namespace {
  tflite::ErrorReporter* error_reporter = nullptr;
  const tflite::Model* model = nullptr;
  tflite::MicroInterpreter* interpreter = nullptr;
  TfLiteTensor* model_input = nullptr;
  TfLiteTensor* model_output = nullptr;

  constexpr int kTensorArenaSize = 32 * 1024;
  uint8_t tensor_arena[kTensorArenaSize];
}

// 📌 Bộ nhớ đệm để lưu 3 lần đo liên tiếp
float history[3][3];  // Lưu trữ dữ liệu trong 3 bước thời gian
int history_count = 0; // Số lần đo đã lưu

void setup() {
  Serial.begin(115200);
  pinMode(RELAY_PIN, OUTPUT);
  dht.begin();

  // 🟢 Khởi tạo TensorFlow Lite
  static tflite::MicroErrorReporter micro_error_reporter;
  error_reporter = &micro_error_reporter;
  
  Serial.println("🔄 Đang tải model...");
  model = tflite::GetModel(nhung_model);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("⚠️ Lỗi: Model không tương thích!");
    return;
  }

  static tflite::AllOpsResolver resolver;
  static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, kTensorArenaSize, error_reporter);
  interpreter = &static_interpreter;

  if (interpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("⚠️ Lỗi: Không thể cấp phát bộ nhớ Tensor!");
    return;
  }

  model_input = interpreter->input(0);
  model_output = interpreter->output(0);

  Serial.println("✅ Model đã sẵn sàng!");
}

void update_history(float temperature, float humidity, float soilMoisture) {
  for (int i = 0; i < 2; i++) {  
    history[i][0] = history[i + 1][0];
    history[i][1] = history[i + 1][1];
    history[i][2] = history[i + 1][2];
  }
  history[2][0] = temperature;
  history[2][1] = humidity;
  history[2][2] = soilMoisture;

  if (history_count < 3) {
    history_count++;
  }
}

void loop() {
  // 📌 Đọc dữ liệu từ cảm biến
  float temperature = dht.readTemperature();  
  float humidity = dht.readHumidity();        
  int soilMoistureRaw = analogRead(SOIL_MOISTURE_PIN);
  float soilMoisture = map(soilMoistureRaw, 0, 4095, 0, 100); 

  // Kiểm tra dữ liệu cảm biến hợp lệ
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("⚠️ Lỗi đọc dữ liệu từ cảm biến!");
    return;
  }

  // Cập nhật dữ liệu vào bộ nhớ đệm
  update_history(temperature, humidity, soilMoisture);

  // Chờ đủ 3 lần đo trước khi đưa vào mô hình
  if (history_count < 3) {
    Serial.println("🔄 Chờ thu thập đủ dữ liệu...");
    delay(5000);
    return;
  }

  Serial.println("------ DỮ LIỆU ĐẦU VÀO ------");
  for (int i = 0; i < 3; i++) {
    Serial.printf("[%d] 🌡️ %.2f°C | 💧 %.2f%% | 🌱 %.2f%%\n", i+1, history[i][0], history[i][1], history[i][2]);
  }

  // 📌 CHUẨN HÓA DỮ LIỆU TRƯỚC KHI ĐƯA VÀO MODEL
  for (int i = 0; i < 3; i++) {
    model_input->data.f[i * 3]     = (history[i][0] - 30.4) / 0.58;
    model_input->data.f[i * 3 + 1] = (history[i][1] - 61.99) / 5.24;
    model_input->data.f[i * 3 + 2] = (history[i][2] - 2712.95) / 179.15;
  }

  // 📌 Thực hiện suy luận (inference)
  TfLiteStatus invoke_status = interpreter->Invoke();
  if (invoke_status != kTfLiteOk) {
    Serial.println("⚠️ Lỗi: Model không thể dự đoán!");
    return;
  }

  // 📌 Lấy kết quả dự đoán (đã chuẩn hóa)
  float predicted_temp = model_output->data.f[0];
  float predicted_humidity = model_output->data.f[1];
  float predicted_soilMoisture = model_output->data.f[2];

  // 📌 GIẢI CHUẨN HÓA KẾT QUẢ DỰ ĐOÁN
  predicted_temp = (predicted_temp * 5.0) + 25.0;
  predicted_humidity = (predicted_humidity * 10.0) + 60.0;
  predicted_soilMoisture = (predicted_soilMoisture * 15.0) + 40.0;

  Serial.println("------ DỰ ĐOÁN ------");
  Serial.printf("🔮 Nhiệt độ dự đoán: %.2f°C\n", predicted_temp);
  Serial.printf("🔮 Độ ẩm không khí dự đoán: %.2f%%\n", predicted_humidity);
  Serial.printf("🔮 Độ ẩm đất dự đoán: %.2f%%\n", predicted_soilMoisture);
  Serial.println("----------------------");

  // 📌 ĐIỀU KHIỂN RELAY
  if (soilMoisture < 30.0 && temperature > 25.0) {
    Serial.println("💧 Độ ẩm đất thấp! BẬT relay để tưới cây.");
    digitalWrite(RELAY_PIN, HIGH);
  } else {
    Serial.println("✅ Độ ẩm đủ! TẮT relay.");
    digitalWrite(RELAY_PIN, LOW);
  }

  delay(5000); // Cập nhật dữ liệu mỗi 5 giây
}
