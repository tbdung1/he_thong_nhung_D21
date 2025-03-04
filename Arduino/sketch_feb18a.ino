#include <DHT.h>

#define DHTPIN 19          // Chân kết nối DHT11
#define soilPin 32         // Chân đo độ ẩm đất
#define relayPin 23        // Chân điều khiển relay

DHT dht(DHTPIN, DHT11);

void setup() {
  Serial.begin(115200);
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);
  dht.begin(); 
}

void loop() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  int soilMoisture = 4095 - analogRead(soilPin);

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Lỗi đọc dữ liệu từ cảm biến!");
    return;
  }

  Serial.print("Temperature:"); Serial.print(temperature); Serial.print("\t");
  Serial.print("Humidity:"); Serial.print(humidity); Serial.print("\t");
  Serial.print("SoilMoisture:"); Serial.println(soilMoisture);

  if (temperature > 27) {
    Serial.println("Bật bơm nước");
    digitalWrite(relayPin, HIGH);
  } else {
    Serial.println("Tắt bơm nước");
    digitalWrite(relayPin, LOW);
  }

  delay(1000);
}