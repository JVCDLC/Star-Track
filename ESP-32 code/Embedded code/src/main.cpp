#include <Arduino.h>
#include "motor_constants.hpp"

int DEC_pwm_min_real = 150; // PWM minimal required to move the motor
volatile long DEC_encoderCount = 0;
volatile uint8_t DEC_prevState = 0;

int RA_pwm_min_real = 150; // PWM minimal required to move the motor
volatile long RA_encoderCount = 0;
volatile uint8_t RA_prevState = 0;

// ==========================
// Control variable
// ==========================
double DEC_target_degree = 0; // modified via motor_angle()
long DEC_target_ticks = 0;
double DEC_prevErrorDeg = 0.0;
double DEC_integralError = 0.0;  // For PID integral term
unsigned long DEC_prevTime = 0;

double RA_target_degree = 0; // modified via motor_angle()
long RA_target_ticks = 0;
double RA_prevErrorDeg = 0.0;
double RA_integralError = 0.0;   // For PID integral term
unsigned long RA_prevTime = 0;

void DEC_drive_motor(int pwm) { // Send PWM to the motor
  digitalWrite(DEC_REN, HIGH);
  digitalWrite(DEC_LEN, HIGH);
  pwm = constrain(pwm, -pwm_max_driver, pwm_max_driver);
  if(pwm > 0){ 
    analogWrite(DEC_RPWM, pwm); 
    analogWrite(DEC_LPWM, 0);
  }
  else if(pwm < 0){ 
    analogWrite(DEC_RPWM, 0); 
    analogWrite(DEC_LPWM, -pwm);
  }
  else {
    analogWrite(DEC_RPWM, 0); 
    analogWrite(DEC_LPWM, 0);
  }

}
void RA_drive_motor(int pwm) { // Send PWM to the motor
  digitalWrite(RA_REN, HIGH);
  digitalWrite(RA_LEN, HIGH);
  pwm = constrain(pwm, -pwm_max_driver, pwm_max_driver);
  if(pwm > 0){ 
    analogWrite(RA_RPWM, pwm); 
    analogWrite(RA_LPWM, 0);
  }
  else if(pwm < 0){ 
    analogWrite(RA_RPWM, 0); 
    analogWrite(RA_LPWM, -pwm);
  }
  else {
    analogWrite(RA_RPWM, 0); 
    analogWrite(RA_LPWM, 0);
  }

}
// ==========================
// Interuption handler for encoder
// ==========================
void ISR_Encoder_DEC() {
  uint8_t a = digitalRead(DEC_ENC_A);
  uint8_t b = digitalRead(DEC_ENC_B);
  uint8_t state = (a << 1) | b;
  uint8_t index = (DEC_prevState << 2) | state;
  DEC_encoderCount += quadTable[index];
  DEC_prevState = state;
}
void ISR_Encoder_RA() {
  uint8_t a = digitalRead(RA_ENC_A);
  uint8_t b = digitalRead(RA_ENC_B);
  uint8_t state = (a << 1) | b;
  uint8_t index = (RA_prevState << 2) | state;
  RA_encoderCount += quadTable[index];
  RA_prevState = state;
}
// ==========================
// Read encoder value and make sure no interrupt create problem
// ==========================
long DEC_encoder_reading() {
  noInterrupts();
  long pos = DEC_encoderCount;
  interrupts();
  return pos;
}
long RA_encoder_reading() {
  noInterrupts();
  long pos = RA_encoderCount;
  interrupts();
  return pos;
}

// ==========================
// Find minimum PWM to rotate motor
// ==========================
int DEC_find_minimum_PWM() {
  int step = 1;
  int pwm = 50 - step;

  while (pwm <= pwm_max_driver) {
    pwm += step;

    long startPos = DEC_encoder_reading();
    DEC_drive_motor(pwm);

    Serial.print("Testing DEC PWM: ");
    Serial.println(pwm);

    delay(200); // un peu plus long

    long delta = abs(DEC_encoder_reading() - startPos);
    if (delta >= 5) {
      DEC_drive_motor(0);
      delay(200);
      return pwm;
    }
    else if(pwm > DEC_pwm_start_max) {
      Serial.println("PWM too high, retest...");
      Serial.println("Check if nothing is blocking the motor and if the motor is properly powered");
      delay(2000);
      pwm = pwm_start_test;
    }
  }

  DEC_drive_motor(0);
  return pwm_max_driver;
}
int RA_find_minimum_PWM() {
  int step = 1;
  int pwm = 50 - step;

  while (pwm <= pwm_max_driver) {
    pwm += step;

    long startPos = RA_encoder_reading();
    RA_drive_motor(pwm);

    Serial.print("Testing RA PWM: ");
    Serial.println(pwm);

    delay(300); // un peu plus long

    long delta = abs(RA_encoder_reading() - startPos);
    if (delta >= 5) {
      RA_drive_motor(0);
      delay(200);
      return pwm;
    }
    else if(pwm > RA_pwm_start_max) {
      Serial.println("PWM too high, retest...");
      Serial.println("Check if nothing is blocking the motor and if the motor is properly powered");
      delay(2000);
      pwm = pwm_start_test;
    }
  }

  RA_drive_motor(0);
  return pwm_max_driver;
}

// ==========================
// Find minimum PWM three time and use the max value for both motors 
// ==========================
void calibrate_minimum_PWM() {
  DEC_pwm_min_real = 0;
  for(int i=0; i<repetitions; i++) {
    int DEC_pwm = 0;
    Serial.print("Test "); Serial.print(i+1); Serial.println(" off PWM minimal...");
    DEC_pwm = DEC_find_minimum_PWM();
    if(DEC_pwm > DEC_pwm_min_real) {
      DEC_pwm_min_real = DEC_pwm;
    }
  }
  RA_pwm_min_real = 0;
  for(int i=0; i<repetitions; i++) {
    int RA_pwm = 0;
    Serial.print("Test "); Serial.print(i+1); Serial.println(" off PWM minimal...");
    RA_pwm = RA_find_minimum_PWM();
    if(RA_pwm > RA_pwm_min_real) {
      RA_pwm_min_real = RA_pwm;
    }
  }
  Serial.print("DEC PWM detecte: "); Serial.println(DEC_pwm_min_real);
  Serial.print("RA PWM detecte: "); Serial.println(RA_pwm_min_real);
  delay(200);
  
}

// ==========================
// Motor control - DEC motor with PID
// ==========================
void DEC_update_motor() {
  unsigned long now = millis();
  double dt = (now - DEC_prevTime) / 1000.0; // seconds
  if (dt <= 0) dt = 0.001; // safety to avoid division by zero

  // Read current position
  long pos = DEC_encoder_reading();
  long error_ticks = DEC_target_ticks - pos;
  double error_deg = error_ticks * 360.0 / TICKS_PER_REV / DEC_gearbox;

  // P term (proportional)
  double p_term = DEC_Kp * error_deg;

  // I term (integral) - accumulate error
  DEC_integralError += error_deg * dt;
  if (DEC_integralError > 100) DEC_integralError = 100;    // Anti-windup
  if (DEC_integralError < -100) DEC_integralError = -100;
  double i_term = DEC_Ki * DEC_integralError;

  // D term (derivative)
  double d_error = (error_deg - DEC_prevErrorDeg) / dt;
  double d_term = DEC_Kd * d_error;

  // Calculate PWM
  double pwm_f = p_term + i_term + d_term;
  int pwm = (int)pwm_f;
  pwm = constrain(pwm, -pwm_max_driver, pwm_max_driver);

  // Minimal PWM to overcome friction
  if (pwm > 0 && pwm < DEC_pwm_min_real) {
    pwm = DEC_pwm_min_real;
    DEC_integralError = 0;
  }
  if (pwm < 0 && pwm > -DEC_pwm_min_real) {
    pwm = -DEC_pwm_min_real;
    DEC_integralError = 0;
  }

  DEC_drive_motor(pwm);

  // Debug output
  char buf_dec[128];
  snprintf(buf_dec, sizeof(buf_dec), "DEC - Target: %.2f° | Error: %.3f° | PWM: %d", DEC_target_degree, error_deg, pwm);
  //Serial.println(buf_dec);

  // Save state
  DEC_prevErrorDeg = error_deg;
  DEC_prevTime = now;
}

// ==========================
// Motor control - RA motor with PID
// ==========================
void RA_update_motor() {
  unsigned long now = millis();
  double dt = (now - RA_prevTime) / 1000.0; // seconds
  if (dt <= 0) dt = 0.001; // safety

  // Read current position
  long pos = RA_encoder_reading();
  long error_ticks = RA_target_ticks - pos;
  double error_deg = error_ticks * 360.0 / TICKS_PER_REV / RA_gearbox;

  // P term (proportional)
  double p_term = RA_Kp * error_deg;

  // I term (integral) - accumulate error
  RA_integralError += error_deg * dt;
  if (RA_integralError > 100) RA_integralError = 100;    // Anti-windup
  if (RA_integralError < -100) RA_integralError = -100;
  double i_term = RA_Ki * RA_integralError;

  // D term (derivative)
  double d_error = (error_deg - RA_prevErrorDeg) / dt;
  double d_term = RA_Kd * d_error;

  // Calculate PWM
  double pwm_f = p_term + i_term + d_term;
  int pwm = (int)pwm_f;
  pwm = constrain(pwm, -pwm_max_driver, pwm_max_driver);

  // Minimal PWM to overcome friction
  if (pwm > 0 && pwm < RA_pwm_min_real) {
    pwm = RA_pwm_min_real;
    RA_integralError = 0;
  }
  if (pwm < 0 && pwm > -RA_pwm_min_real) {
    pwm = -RA_pwm_min_real;
    RA_integralError = 0;
  }

  RA_drive_motor(pwm);

  // Debug output
  char buf_ra[128];
  snprintf(buf_ra, sizeof(buf_ra), "RA  - Target: %.2f° | Error: %.3f° | PWM: %d", RA_target_degree, error_deg, pwm);
  //Serial.println(buf_ra);

  // Save state
  RA_prevErrorDeg = error_deg;
  RA_prevTime = now;
}

void update_motor() {
  DEC_update_motor();
  RA_update_motor();
}

// ==========================
// Set motor target angle
// ==========================
void set_motor_dec_angle(double deg) {
  DEC_target_degree = deg;
  DEC_target_ticks = DEC_target_degree * DEC_gearbox * TICKS_PER_REV / 360.0;
  DEC_integralError = 0.0; // Reset integral term on new target
}

void set_motor_ra_angle(double deg) {
  RA_target_degree = deg;
  RA_target_ticks = RA_target_degree * RA_gearbox * TICKS_PER_REV / 360.0;
  
  RA_integralError = 0.0; // Reset integral term on new target
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
  // ==========================
  // Setup DEC motor
  // ==========================
  pinMode(DEC_ENC_A, INPUT_PULLUP);
  pinMode(DEC_ENC_B, INPUT_PULLUP);
  DEC_prevState = (digitalRead(DEC_ENC_A) << 1) | digitalRead(DEC_ENC_B);
  attachInterrupt(digitalPinToInterrupt(DEC_ENC_A), ISR_Encoder_DEC, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DEC_ENC_B), ISR_Encoder_DEC, CHANGE);
  DEC_encoderCount = 0;

  pinMode(DEC_REN, OUTPUT);
  pinMode(DEC_LEN, OUTPUT);
  digitalWrite(DEC_REN, HIGH);
  digitalWrite(DEC_LEN, HIGH);


  pinMode(DEC_RPWM, OUTPUT);
  pinMode(DEC_LPWM, OUTPUT);

  // ==========================
  // Setup RA motor
  // ==========================
  pinMode(RA_ENC_A, INPUT_PULLUP);
  pinMode(RA_ENC_B, INPUT_PULLUP);
  RA_prevState = (digitalRead(RA_ENC_A) << 1) | digitalRead(RA_ENC_B);
  attachInterrupt(digitalPinToInterrupt(RA_ENC_A), ISR_Encoder_RA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RA_ENC_B), ISR_Encoder_RA, CHANGE);
  RA_encoderCount = 0;

  pinMode(RA_REN, OUTPUT);
  pinMode(RA_LEN, OUTPUT);
  digitalWrite(RA_REN, HIGH);
  digitalWrite(RA_LEN, HIGH);

  pinMode(RA_RPWM, OUTPUT);
  pinMode(RA_LPWM, OUTPUT);

  Serial.println("Systeme pret - detection automatique du PWM minimal");
  // Calibrate minimal PWM for both motors (updates DEC_pwm_min_real and RA_pwm_min_real)
  calibrate_minimum_PWM();
  Serial.print("PWM minimal DEC: "); Serial.println(DEC_pwm_min_real);
  Serial.print("PWM minimal RA: "); Serial.println(RA_pwm_min_real);

  // Initialize timing to avoid large dt on first loop
  DEC_prevTime = millis();
  RA_prevTime = millis();

  // Initial position
  set_motor_dec_angle(0);
  set_motor_ra_angle(0);

}

// ==========================
// LOOP
// ==========================
void loop() {
  update_motor();
  delay(5); // small delay to not saturate cpu
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    msg.trim(); // remove on necessairy space and \n

    Serial.println("RX: " + msg);

    // cut message on message
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
      } else if (motor == "RIGHT_ASCENSION") {
        set_motor_ra_angle(target_angle);
        Serial.print("ACK REQUEST | Motor: RA | Target: ");
        Serial.println(target_angle, 3);
      } else {
        Serial.println("ERR: unknown motor");
      }
    } else {
      Serial.println("ERR: not a valid command");
    }
  }
}
