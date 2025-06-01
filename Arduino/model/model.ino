#include <Arduino.h>
#include <DHT.h>
#include <TensorFlowLite_ESP32.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"

// 📌 Load model đã convert
#include "nhung_model.h" // Đảm bảo file này tồn tại và đúng tên

// 📌 Cấu hình cảm biến
#define DHTPIN 19
#define DHTTYPE DHT11
#define SOIL_MOISTURE_PIN 32 // Chân Analog đọc độ ẩm đất
#define RELAY_PIN 23

DHT dht(DHTPIN, DHTTYPE);

// 📌 Khai báo TensorFlow Lite
namespace {
  tflite::ErrorReporter* error_reporter = nullptr;
  const tflite::Model* model = nullptr;
  tflite::MicroInterpreter* interpreter = nullptr;
  TfLiteTensor* model_input = nullptr;
  TfLiteTensor* model_output = nullptr;

  constexpr int kTensorArenaSize = 32 * 1024; // Tăng nếu cần
  uint8_t tensor_arena[kTensorArenaSize];
}

// 📌 Các biến toàn cục để lưu trữ dữ liệu và kết quả
float history[3][3]; // Lưu trữ lịch sử: [temp, humid, soil_raw]
float predictedData[3]; // Lưu trữ kết quả dự đoán: [pred_temp, pred_humid, pred_soil_raw]
float temperature, humidity;
int soilMoistureRaw; // Giá trị độ ẩm đất gốc (0-4095) - Sử dụng giá trị này xuyên suốt

// 📌 Queue để truyền dữ liệu giữa các task
// Queue sẽ chứa: [temperature, humidity, soilMoistureRaw]
QueueHandle_t sensorDataQueue;

// --- CÁC HÀM HELPER ---
// (Giữ nguyên hàm normalize và denormalize)
float normalize(float value, float mean, float stdDev) {
  if (stdDev == 0) return value;
  return (value - mean) / stdDev;
}

float denormalize(float value, float mean, float stdDev) {
  return (value * stdDev) + mean;
}
// --- KẾT THÚC HÀM HELPER ---

// 📌 Task 1: Thu thập dữ liệu cảm biến
void taskReadSensors(void *pvParameters) {
  for (;;) {
    // Đọc dữ liệu từ cảm biến
    temperature = dht.readTemperature();
    humidity = dht.readHumidity();
    soilMoistureRaw = analogRead(SOIL_MOISTURE_PIN); // Đọc giá trị raw 0-4095

    // Kiểm tra dữ liệu cảm biến hợp lệ
    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("⚠️ Lỗi đọc dữ liệu từ cảm biến DHT!");
      vTaskDelay(pdMS_TO_TICKS(5000));
      continue;
    }

    // In dữ liệu ra Serial để theo dõi
    Serial.print("Nhiet do: ");
    Serial.print(temperature);
    Serial.print(" *C, ");

    Serial.print("Do am KK: ");
    Serial.print(humidity);
    Serial.print(" %, ");

    Serial.print("Do am dat (Raw): "); // Chỉ hiển thị giá trị Raw
    Serial.println(soilMoistureRaw);

    // Gửi dữ liệu vào queue [temp, humid, soil_raw]
    float sensorData[3] = {temperature, humidity, (float)soilMoistureRaw}; // Gửi giá trị RAW
    if (xQueueSend(sensorDataQueue, sensorData, pdMS_TO_TICKS(100)) != pdTRUE) {
       Serial.println("⚠️ Queue đầy, không thể gửi dữ liệu cảm biến!");
    }

    vTaskDelay(pdMS_TO_TICKS(10000)); // Chạy mỗi 10s
  }
}

// 📌 Task 2: Dự đoán môi trường
void taskPredict(void *pvParameters) {
  float modelInput[3 * 3];
  float receivedSensorData[3];
  int history_count = 0;

  // Mean & Std Dev cho chuẩn hóa (Sử dụng cho giá trị RAW của độ ẩm đất)
  const float temp_mean = 30.34, temp_std = 0.58;
  const float humid_mean = 61.99, humid_std = 5.24;
  const float soil_raw_mean = 2712.95, soil_raw_std = 179.15; // Mean/Std cho giá trị RAW

  for (;;) {
    if (xQueueReceive(sensorDataQueue, receivedSensorData, portMAX_DELAY) == pdTRUE) {
      // Cập nhật lịch sử (lưu temp, humid, soil_raw)
      for (int i = 0; i < 2; i++) {
        history[i][0] = history[i + 1][0];
        history[i][1] = history[i + 1][1];
        history[i][2] = history[i + 1][2]; // Soil Moisture RAW
      }
      history[2][0] = receivedSensorData[0];
      history[2][1] = receivedSensorData[1];
      history[2][2] = receivedSensorData[2]; // Lưu giá trị RAW

      if (history_count < 3) {
        history_count++;
        Serial.printf("🔄 Đã nhận %d/3 mẫu. Chờ thu thập đủ dữ liệu...\n", history_count);
        continue;
      }

      Serial.println("------ DỮ LIỆU ĐẦU VÀO MODEL (RAW) ------");
      for (int i = 0; i < 3; i++) {
        Serial.printf("[%d] 🌡️ %.2f°C | 💧 %.2f%% | 🌱 %.0f (Raw)\n", i + 1, history[i][0], history[i][1], history[i][2]);
      }

      // Chuẩn hóa dữ liệu đầu vào
      for (int i = 0; i < 3; i++) {
        modelInput[i * 3 + 0] = normalize(history[i][0], temp_mean, temp_std);
        modelInput[i * 3 + 1] = normalize(history[i][1], humid_mean, humid_std);
        modelInput[i * 3 + 2] = normalize(history[i][2], soil_raw_mean, soil_raw_std); // Độ ẩm đất RAW
      }

       if (model_input == nullptr) { /*...*/ continue; } // Giữ nguyên kiểm tra lỗi
      for (int i = 0; i < 9; i++) {
        model_input->data.f[i] = modelInput[i];
      }

      // Thực hiện suy luận
      TfLiteStatus invoke_status = interpreter->Invoke();
      if (invoke_status != kTfLiteOk) {
        error_reporter->Report("Invoke failed");
        Serial.println("⚠️ Lỗi: Model không thể dự đoán!");
        vTaskDelay(pdMS_TO_TICKS(5000));
        continue;
      }

       if (model_output == nullptr) { /*...*/ continue; } // Giữ nguyên kiểm tra lỗi
      // Lấy kết quả dự đoán (đã chuẩn hóa)
      float pred_norm_temp = model_output->data.f[0];
      float pred_norm_humid = model_output->data.f[1];
      float pred_norm_soil_raw = model_output->data.f[2];

      // Giải chuẩn hóa kết quả dự đoán
      predictedData[0] = denormalize(pred_norm_temp, temp_mean, temp_std);         // Predicted Temp
      predictedData[1] = denormalize(pred_norm_humid, humid_mean, humid_std);       // Predicted Humidity
      predictedData[2] = denormalize(pred_norm_soil_raw, soil_raw_mean, soil_raw_std); // Predicted Soil RAW

      // Đảm bảo giá trị dự đoán hợp lệ
      predictedData[1] = max(0.0f, predictedData[1]);
      predictedData[1] = min(100.0f, predictedData[1]);
      predictedData[2] = max(0.0f, predictedData[2]); // Đảm bảo raw >= 0
      predictedData[2] = min(4095.0f, predictedData[2]); // Đảm bảo raw <= 4095 (nếu cần)


      // In kết quả dự đoán ra Serial
      Serial.println("------ DỰ ĐOÁN ------");
      Serial.printf("🔮 Nhiệt độ dự đoán: %.2f°C\n", predictedData[0]);
      Serial.printf("🔮 Độ ẩm không khí dự đoán: %.2f%%\n", predictedData[1]);
      Serial.printf("🔮 Độ ẩm đất dự đoán (Raw): %.0f\n", predictedData[2]); // Hiển thị giá trị Raw dự đoán
      Serial.println("----------------------");

    }
  }
}

// 📌 Task 3: Điều khiển hệ thống tưới (SỬ DỤNG GIÁ TRỊ RAW)
void taskControlWatering(void *pvParameters) {
  // --- NGƯỠNG DÙNG GIÁ TRỊ RAW (0-4095) ---
  // Quan trọng: Xác định ngưỡng này dựa trên cảm biến và nhu cầu thực tế.
  // Giả sử giá trị RAW CÀNG CAO thì CÀNG KHÔ (phổ biến với cảm biến điện dung).
  // Ví dụ: Tưới nếu giá trị raw vượt quá 2900.
  int soilMoistureRawThreshold = 2900; // <-- THAY ĐỔI NGƯỠNG NÀY CHO PHÙ HỢP
  float temperatureThreshold = 35.0;  // Ngưỡng nhiệt độ (ví dụ)

  for (;;) {
    // Sử dụng biến toàn cục:
    // temperature (hiện tại)
    // soilMoistureRaw (hiện tại, 0-4095)

    // Ra quyết định tưới hay không dựa trên giá trị RAW
    bool shouldWater = false;

    // --- Logic điều khiển (VÍ DỤ - Dùng giá trị RAW) ---
    // 1. Tưới nếu độ ẩm đất hiện tại (RAW) THẤP HƠN ngưỡng (tức là khô hơn)
    if (soilMoistureRaw > soilMoistureRawThreshold) {
      shouldWater = true;
      Serial.printf("💡 Lý do tưới: Độ ẩm Raw hiện tại (%d) > Ngưỡng Raw (%d)\n", soilMoistureRaw, soilMoistureRawThreshold);
    }
    // 2. Hoặc tưới nếu nhiệt độ hiện tại cao
    else if (temperature > temperatureThreshold) {
       shouldWater = true;
       Serial.printf("💡 Lý do tưới: Nhiệt độ cao (%.1f°C)\n", temperature);
    }
    // --- Kết thúc Logic điều khiển ---


    if (shouldWater) {
      digitalWrite(RELAY_PIN, HIGH);
      Serial.println("💧 BẬT relay để tưới cây (dựa trên giá trị hiện tại).");
      vTaskDelay(pdMS_TO_TICKS(15000)); // Thời gian tưới
      digitalWrite(RELAY_PIN, LOW);
      Serial.println("✅ Ngừng tưới cây.");
    } else {
      digitalWrite(RELAY_PIN, LOW);
      // Cập nhật log
      Serial.printf("✅ Độ ẩm đủ (Hiện tại Raw: %d). Nhiệt độ tốt (Hiện tại: %.2f°C). Không tưới.\n", soilMoistureRaw, temperature);
    }
    vTaskDelay(pdMS_TO_TICKS(10000)); // Chờ 10s
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  pinMode(SOIL_MOISTURE_PIN, INPUT);
  dht.begin();
  delay(2000);

  // 🟢 Khởi tạo TensorFlow Lite (Giữ nguyên)
  static tflite::MicroErrorReporter micro_error_reporter;
  error_reporter = &micro_error_reporter;

  Serial.println("🔄 Đang tải model TFLite...");
  model = tflite::GetModel(nhung_model);
  if (model->version() != TFLITE_SCHEMA_VERSION) { /*...*/ return; } // Giữ nguyên kiểm tra lỗi

  static tflite::AllOpsResolver resolver;
  static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensor_arena, kTensorArenaSize, error_reporter);
  interpreter = &static_interpreter;

  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) { /*...*/ return; } // Giữ nguyên kiểm tra lỗi

  model_input = interpreter->input(0);
  model_output = interpreter->output(0);

   // In thông tin Model (Giữ nguyên)
   Serial.println("--- Model Info ---");
   Serial.printf("Input dims: %d", model_input->dims->size);
   for(int i=0; i<model_input->dims->size; ++i) Serial.printf(" [%d]", model_input->dims->data[i]);
   Serial.printf(", Type: %d\n", model_input->type);
   Serial.printf("Output dims: %d", model_output->dims->size);
   for(int i=0; i<model_output->dims->size; ++i) Serial.printf(" [%d]", model_output->dims->data[i]);
   Serial.printf(", Type: %d\n", model_output->type);
   Serial.println("------------------");


  Serial.println("✅ Model đã sẵn sàng!");

  // 📌 Tạo queue (Giữ nguyên)
  sensorDataQueue = xQueueCreate(5, sizeof(float) * 3);
  if (sensorDataQueue == NULL) { /*...*/ return; } // Giữ nguyên kiểm tra lỗi
  else { Serial.println("✅ Queue đã được tạo."); }


  // 📌 Tạo các task FreeRTOS (Giữ nguyên stack sizes và kiểm tra lỗi)
  BaseType_t taskCreateResult;
  taskCreateResult = xTaskCreate(taskReadSensors, "ReadSensors", 4096, NULL, 1, NULL);
  if (taskCreateResult != pdPASS) { Serial.println("⚠️ Lỗi: Không thể tạo task ReadSensors!"); } else { Serial.println("✅ Task ReadSensors đã tạo."); }

  taskCreateResult = xTaskCreate(taskPredict, "Predict", 8192, NULL, 2, NULL);
   if (taskCreateResult != pdPASS) { Serial.println("⚠️ Lỗi: Không thể tạo task Predict!"); } else { Serial.println("✅ Task Predict đã tạo."); }

  taskCreateResult = xTaskCreate(taskControlWatering, "ControlWatering", 4096, NULL, 1, NULL);
  if (taskCreateResult != pdPASS) { Serial.println("⚠️ Lỗi: Không thể tạo task ControlWatering!"); } else { Serial.println("✅ Task ControlWatering đã tạo."); }

   Serial.println("🚀 Hệ thống khởi động hoàn tất! (Sử dụng giá trị Độ ẩm đất RAW)");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(5000));
}