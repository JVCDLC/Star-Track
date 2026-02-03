#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// I2C pins for ESP32-C3 SuperMini (adjust if your wiring differs)
constexpr uint8_t I2C_SDA_PIN = 4;
constexpr uint8_t I2C_SCL_PIN = 3;

// MPU6050 I2C address: 0x68 (default) or 0x69 (AD0 high)
constexpr uint8_t MPU6050_ADDR = 0x68;

Adafruit_MPU6050 mpu;

void scanI2C() {
  Serial.println("I2C scan start...");
  uint8_t count = 0;
  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found 0x");
      if (addr < 16) {
        Serial.print('0');
      }
      Serial.println(addr, HEX);
      ++count;
    }
  }
  if (count == 0) {
    Serial.println("No I2C devices found.");
  }
  Serial.println("I2C scan done.");
}

void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 3000)) {
    delay(10);
  }

  Serial.println("Booting...");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
  scanI2C();

  if (!mpu.begin(MPU6050_ADDR, &Wire)) {
    Serial.println("MPU6050 not found. Check wiring/address.");
    while (true) {
      delay(1000);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU6050 ready.");
}

void loop() {
  static unsigned long lastPrintMs = 0;
  const unsigned long printIntervalMs = 500;
  const unsigned long now = millis();
  if (now - lastPrintMs < printIntervalMs) {
    return;
  }
  lastPrintMs = now;

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  mpu.getEvent(&accel, &gyro, &temp);

  Serial.printf("t=%lu ms | Accel (m/s^2): %.3f, %.3f, %.3f\n",
                now,
                accel.acceleration.x,
                accel.acceleration.y,
                accel.acceleration.z);
}