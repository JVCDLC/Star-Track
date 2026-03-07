#include <Arduino.h>
#include "constants.hpp"
#include "motors.hpp"

// ==========================
// SETUP
// ==========================
void setup() {

  // ==========================
  // SET UP the timers for silent PWM (no buzzing noise)
  // ==========================
  TCCR4B = (TCCR4B & 0b11111000) | 0x01;
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;
  TCCR5B = (TCCR5B & 0b11111000) | 0x01;
  TCCR2B = (TCCR2B & 0b11111000) | 0x01;

  Serial.begin(115200);
  delay(500);

  Serial.println("Systeme pret - detection automatique du PWM minimal");

  motors_init();
  calibrate_minimum_PWM();

  Serial.print("PWM minimal DEC: "); Serial.println(DEC_pwm_min_real);
  Serial.print("PWM minimal RA: "); Serial.println(RA_pwm_min_real);

  DEC_prevTime = millis();
  RA_prevTime = millis();

  set_motor_dec_angle(0);
  set_motor_ra_angle(0);
}

// ==========================
// LOOP
// ==========================
void loop() {

  update_motor();
  delay(5);

  if (Serial.available()) {

    String msg = Serial.readStringUntil('\n');
    msg.trim();

    Serial.println("RX: " + msg);

    int i1 = msg.indexOf(',');
    int i2 = msg.indexOf(',', i1 + 1);

    if (i1 == -1 || i2 == -1) {
      Serial.println("ERR: bad message");
      return;
    }

    String cmd   = msg.substring(0, i1);
    String motor = msg.substring(i1 + 1, i2);
    String value = msg.substring(i2 + 1);

    if (cmd == "REQUEST") {

      double target_angle = value.toFloat();

      if (motor == "DECLIMATION") {

        set_motor_dec_angle(target_angle);

        Serial.print("ACK REQUEST | Motor: DEC | Target: ");
        Serial.println(target_angle, 3);

      }
      else if (motor == "RIGHT_ASCENSION") {

        set_motor_ra_angle(target_angle);

        Serial.print("ACK REQUEST | Motor: RA | Target: ");
        Serial.println(target_angle, 3);

      }
      else {
        Serial.println("ERR: unknown motor");
      }
    }
    else {
      Serial.println("ERR: not a valid command");
    }
  }
}