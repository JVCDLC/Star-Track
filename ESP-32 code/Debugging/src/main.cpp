#include <Arduino.h>
#include "constants.hpp"



void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== RA Motor Simple Test ===");
  
  // Initialize motor pins
  pinMode(RA_RPWM, OUTPUT);
  pinMode(RA_LPWM, OUTPUT);
  pinMode(RA_REN, OUTPUT);
  pinMode(RA_LEN, OUTPUT);
  
  // Initialize encoder pins (optional for simple test)
  pinMode(RA_ENC_A, INPUT_PULLUP);
  pinMode(RA_ENC_B, INPUT_PULLUP);
  
  // Enable the motor driver
  digitalWrite(RA_REN, HIGH);
  digitalWrite(RA_LEN, HIGH);
  
  Serial.println("Motor initialized. Starting rotation...");
}

void loop() {
  digitalWrite(RA_REN, HIGH);
  digitalWrite(RA_LEN, HIGH);

  // Rotate motor clockwise with PWM 200
  Serial.println("Rotating clockwise with PWM 200...");
  analogWrite(RA_RPWM, 200);
  analogWrite(RA_LPWM, 0);
  delay(3000);
  
  // Stop
  Serial.println("Stopping motor...");
  analogWrite(RA_RPWM, 0);
  analogWrite(RA_LPWM, 0);
  delay(1000);
  
  // Rotate motor counterclockwise with PWM 200
  Serial.println("Rotating counterclockwise with PWM 200...");
  analogWrite(RA_RPWM, 0);
  analogWrite(RA_LPWM, 1000);
  delay(3000);
  
  // Stop
  Serial.println("Stopping motor...");
  analogWrite(RA_RPWM, 0);
  analogWrite(RA_LPWM, 0);
  delay(1000);
}

