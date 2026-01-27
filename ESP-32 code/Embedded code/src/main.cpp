#include <Arduino.h>

void setup() {
  Serial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    Serial.println("ACK:" + msg);
  }
}