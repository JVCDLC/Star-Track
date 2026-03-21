#include <Arduino.h>
#include "constants.hpp"
#include "motors.hpp"
#include "switchs.hpp"
volatile uint8_t last_portB = 0;
volatile uint8_t FOCUS_prevState = 0;
volatile long FOCUS_encoderCount = 0;


// interruption for encoder and switchs on port B (pins 50 to 53 for encoder, pins 6 and 7 for switchs)
ISR(PCINT0_vect)
{
    uint8_t current = PINB;
    uint8_t changed = current ^ last_portB;
    last_portB = current;

    // Switchs PB6/PB7
    if (changed & (1 << 6)) switch9_triggered  = !(current & (1 << 6));
    if (changed & (1 << 7)) switch10_triggered = !(current & (1 << 7));

    // Encoder FOCUS PB2/PB3 (pins 50/51)
    uint8_t a = (current & (1 << 2)) >> 2; // pin 50
    uint8_t b = (current & (1 << 3)) >> 3; // pin 51
    uint8_t state = (b << 1) | a;          // MSB = b, LSB = a
    uint8_t index = (FOCUS_prevState << 2) | state;
    FOCUS_encoderCount += quadTable[index];
    FOCUS_prevState = state;
}

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

  Serial.println("Systeme ready - detection of switchs and motors minimum PWM...");
  switches_init();
  motors_init();
  Serial.println("Finding angle range for DEC motor...");
  DEC_find_angle_range();
  Serial.println("calibration of minimum PWM for the motors...");
  //calibrate_minimum_PWM();

  Serial.print("PWM minimal DEC: "); Serial.println(DEC_pwm_min_real);
  Serial.print("PWM minimal RA: "); Serial.println(RA_pwm_min_real);
  Serial.print("PWM minimal FOCUS: "); Serial.println(FOCUS_pwm_min_real);

  DEC_prevTime = millis();
  RA_prevTime = millis();
  FOCUS_prevTime = millis();
  set_motor_dec_angle(0);
  set_motor_ra_angle(0);
  set_motor_focus_angle(0);
  delay(500);
}

  static unsigned long lastChange = 0;
  static bool at360 = false;

// ==========================
// LOOP
// ==========================
void loop() {
  //Serial.println("state of switchs: " + String(switch1_triggered)+ String(switch2_triggered)+ String(switch3_triggered)+ String(switch4_triggered)+ String(switch5_triggered)+ String(switch6_triggered)+ String(switch7_triggered)+ String(switch8_triggered)+ String(switch9_triggered)+ String(switch10_triggered));

  //Serial.println(DEC_encoder_reading()+String(" ")+RA_encoder_reading()+String(" ")+FOCUS_encoder_reading());
  unsigned long now = millis();
  /*
  // change target angle of focus every 5 seconds for testing
  if (now - lastChange >= 6000) {

    lastChange = now;
    at360 = !at360;

    if (at360){
      //set_motor_focus_angle(3600);
      set_motor_ra_angle(360);
      set_motor_dec_angle(360);
    }
    else{
      //set_motor_focus_angle(0);
      set_motor_ra_angle(0);
      set_motor_dec_angle(0);
    }
  }
  */
  // update_motor() is called to update the pwm of the motors according to the target 
  // angle and the encoder reading, it is important to call it as often as possible 
  // to have a good control of the motors
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