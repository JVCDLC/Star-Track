#include <Arduino.h>
#include "constants.hpp"
#include "motors.hpp"

int DEC_pwm_min_real = 150;
volatile long DEC_encoderCount = 0;
volatile uint8_t DEC_prevState = 0;

int RA_pwm_min_real = 150;
volatile long RA_encoderCount = 0;
volatile uint8_t RA_prevState = 0;

int FOCUS_pwm_min_real = 50;


// ==========================
// Control variable
// ==========================
double DEC_target_degree = 0;
long DEC_target_ticks = 0;
double DEC_prevErrorDeg = 0.0;
double DEC_integralError = 0.0;
unsigned long DEC_prevTime = 0;

double RA_target_degree = 0;
long RA_target_ticks = 0;
double RA_prevErrorDeg = 0.0;
double RA_integralError = 0.0;
unsigned long RA_prevTime = 0;

double FOCUS_target_degree = 0;
long FOCUS_target_ticks = 0;
double FOCUS_prevErrorDeg = 0.0;
double FOCUS_integralError = 0.0;
unsigned long FOCUS_prevTime = 0;

long DEC_min_angle = 0;
long DEC_max_angle = 0;
long RA_min_angle = 0;
long RA_max_angle = 0;
long FOCUS_min_angle = 0;
long FOCUS_max_angle = 0;
// ==========================
// Motor driving
// ==========================
void DEC_drive_motor(int pwm) {
  // Enable the motor driver, it need to be enable to work.
  digitalWrite(DEC_REN, HIGH);
  digitalWrite(DEC_LEN, HIGH);

  // Constrain PWM to the allowed range
  pwm = constrain(pwm, -pwm_max_driver, pwm_max_driver);

  if((switch4_triggered && pwm > 0)||(switch5_triggered && pwm < 0)){// STOP if switch 4 or 5 is triggered and the motor is trying to move in the direction that would trigger it
    pwm = 0;
  }

  if (pwm > 0) {
    analogWrite(DEC_RPWM, pwm);
    analogWrite(DEC_LPWM, 0);
  }
  else if (pwm < 0) {
    analogWrite(DEC_RPWM, 0);
    analogWrite(DEC_LPWM, -pwm);
  }
  else {
    analogWrite(DEC_RPWM, 0);
    analogWrite(DEC_LPWM, 0);
  }
}

void RA_drive_motor(int pwm) {
  // Enable the motor driver, it need to be enable to work.
  digitalWrite(RA_REN, HIGH);
  digitalWrite(RA_LEN, HIGH);

  // Constrain PWM to the allowed range
  pwm = constrain(pwm, -pwm_max_driver, pwm_max_driver);

  if((switch2_triggered && pwm > 0)||(switch3_triggered && pwm < 0)){// STOP if switch 2 or 3 is triggered and the motor is trying to move in the direction that would trigger it
    pwm = 0;
  }
  if (pwm > 0) {
    analogWrite(RA_RPWM, pwm);
    analogWrite(RA_LPWM, 0);
  }
  else if (pwm < 0) {
    analogWrite(RA_RPWM, 0);
    analogWrite(RA_LPWM, -pwm);
  }
  else {
    analogWrite(RA_RPWM, 0);
    analogWrite(RA_LPWM, 0);
  }
}

void FOCUS_drive_motor(int pwm) {

  pwm = -constrain(pwm, -pwm_max_driver, pwm_max_driver);
  if(switch1_triggered && pwm > 0){// STOP if switch 1 is triggered and the motor is trying to move in the direction that would trigger it
    pwm = 0;
  }
  if (pwm > 0) {
    digitalWrite(FOCUS_DIR, HIGH);
    analogWrite(FOCUS_PWM, pwm);
  }
  else if (pwm < 0) {
    digitalWrite(FOCUS_DIR, LOW);
    analogWrite(FOCUS_PWM, -pwm);
  }
  else {
    analogWrite(FOCUS_PWM, 0);
  }
}

// ==========================
// Encoder interrupts
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
// Encoder reading. Theses functions are used to read encoder values without having access problems
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

long FOCUS_encoder_reading() {

    
    noInterrupts();
    long pos = FOCUS_encoderCount;
    interrupts();
    //Serial.print("FOCUS encoder count: "); Serial.println(pos);

    return pos;
}

// ==========================
// Find angle range for the motors using limit switchs
// ==========================
void DEC_find_angle_range(){
  float ratio = 0.65;
  // move in one direction until switch is triggered
  while (!switch4_triggered) {
    Serial.println(switch4_triggered);
    DEC_drive_motor(ratio*pwm_max_driver);
    delay(10);
  }
  
  DEC_drive_motor(0);
  Serial.println("Switch 2 triggered, minimum angle reached");
  DEC_min_angle = DEC_encoder_reading();
  // move a bit to do a second passage
  Serial.println("Move a bit to do a second passage");
  DEC_drive_motor(-ratio*pwm_max_driver);
  delay(500);
  // second passage to be sure the initial value  is correct
  while (!switch4_triggered) {
    DEC_drive_motor(ratio*DEC_pwm_start_max);
    delay(10);
  }
  DEC_drive_motor(0);
  DEC_min_angle = DEC_encoder_reading();



  while (!switch5_triggered) {
    DEC_drive_motor(-ratio*pwm_max_driver);
    delay(10);
  }
  // move a bit to do a second passage
  DEC_drive_motor(ratio*pwm_max_driver);
  delay(1000);
  // second passage to be sure the initial value  is correct
  while (!switch5_triggered) {
    DEC_drive_motor(-ratio*DEC_pwm_start_max);
    delay(10);
  }
  DEC_drive_motor(0);
  DEC_max_angle = DEC_encoder_reading();

  Serial.print("DEC angle range in ticks: ");
  Serial.print(DEC_min_angle);
  Serial.print(" to ");
  Serial.println(DEC_max_angle);
  long total_ticks = DEC_max_angle - DEC_min_angle;
  float middle_tick = DEC_min_angle + total_ticks / 2.0;
  Serial.println(total_ticks);
  set_motor_dec_ticks(middle_tick);
 long pos;
  while (true) {
    pos = DEC_encoder_reading();

    if (abs(pos - middle_tick) <= 10)
      break;

    DEC_update_motor();
    delay(10);
  }
  DEC_encoderCount=0;
}

void RA_find_angle_range(){
  // move in one direction until switch is triggered
  while (switch2_triggered) {
    RA_drive_motor(pwm_max_driver);
    delay(10);
  }
  
  RA_drive_motor(0);
  Serial.println("Switch 2 triggered, minimum angle reached");
  RA_min_angle = RA_encoder_reading();
  // move a bit to do a second passage
  Serial.println("Move a bit to do a second passage");
  RA_drive_motor(-pwm_max_driver);
  delay(1000);
  // second passage to be sure the initial value  is correct
  while (switch2_triggered) {
    RA_drive_motor(RA_pwm_start_max);
    delay(10);
  }
  RA_drive_motor(0);
  RA_min_angle = RA_encoder_reading();



  while (switch3_triggered) {
    RA_drive_motor(-RA_pwm_start_max);
    delay(10);
  }
  // move a bit to do a second passage
  RA_drive_motor(RA_pwm_start_max);
  delay(1000);
  // second passage to be sure the initial value  is correct
  while (switch3_triggered) {
    RA_drive_motor(-RA_pwm_start_max);
    delay(10);
  }
  RA_drive_motor(0);
  RA_min_angle = RA_encoder_reading();

  Serial.print("RA angle range in ticks: ");
  Serial.print(RA_min_angle);
  Serial.print(" to ");
  Serial.println(RA_max_angle);
  int total_ticks = abs(RA_max_angle - RA_min_angle);
  Serial.println(total_ticks);

}

void FOCUS_find_angle_range(){}

// ==========================
// Find minimum PWM
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

    delay(200);

    long delta = abs(DEC_encoder_reading() - startPos);

    if (delta >= 5) {

      DEC_drive_motor(0);
      delay(200);
      return pwm;
    }

    else if (pwm > DEC_pwm_start_max) {

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

    delay(200);

    long delta = abs(RA_encoder_reading() - startPos);

    if (delta >= 5) {

      RA_drive_motor(0);
      delay(200);
      return pwm;
    }

    else if (pwm > RA_pwm_start_max) {

      Serial.println("PWM too high, retest...");
      Serial.println("Check if nothing is blocking the motor and if the motor is properly powered");

      delay(2000);
      pwm = pwm_start_test;
    }
  }

  RA_drive_motor(0);
  return pwm_max_driver;
}

int FOCUS_find_minimum_PWM() {

  int step = 1;
  int pwm = 50 - step;

  while (pwm <= pwm_max_driver) {

    pwm += step;

    long startPos = FOCUS_encoder_reading();
    FOCUS_drive_motor(pwm);

    Serial.print("Testing FOCUS PWM: ");
    Serial.println(pwm);

    delay(200);

    long delta = abs(FOCUS_encoder_reading() - startPos);

    if (delta >= 5) {

      FOCUS_drive_motor(0);
      delay(200);
      return pwm;
    }

    else if (pwm > FOCUS_pwm_start_max) {

      Serial.println("PWM too high, retest...");
      delay(2000);
      pwm = pwm_start_test;
    }
  }

  FOCUS_drive_motor(0);
  return pwm_max_driver;
}

// ==========================
// Calibration
// ==========================
void calibrate_minimum_PWM() {
  DEC_pwm_min_real = 0;
/*
  for (int i = 0; i < repetitions; i++) {

    int DEC_pwm = 0;

    Serial.print("Test ");
    Serial.print(i + 1);
    Serial.println(" off PWM minimal...");

    DEC_pwm = DEC_find_minimum_PWM();

    if (DEC_pwm > DEC_pwm_min_real) {
      DEC_pwm_min_real = DEC_pwm;
    }
  }
  */

  RA_pwm_min_real = 0;

  for (int i = 0; i < repetitions; i++) {

    int RA_pwm = 0;

    Serial.print("Test ");
    Serial.print(i + 1);
    Serial.println(" off PWM minimal...");

    RA_pwm = RA_find_minimum_PWM();

    if (RA_pwm > RA_pwm_min_real) {
      RA_pwm_min_real = RA_pwm;
    }
  }

  FOCUS_pwm_min_real = 0;

  for (int i = 0; i < repetitions; i++) {

    int pwm = 0;

    Serial.print("Test ");
    Serial.print(i + 1);
    Serial.println(" off PWM minimal...");

    pwm = FOCUS_find_minimum_PWM();

    if (pwm > FOCUS_pwm_min_real) {
      FOCUS_pwm_min_real = pwm;
    }
  }

  Serial.print("DEC PWM detecte: ");
  Serial.println(DEC_pwm_min_real);

  Serial.print("RA PWM detecte: ");
  Serial.println(RA_pwm_min_real);

  Serial.print("FOCUS PWM detecte: ");
  Serial.println(FOCUS_pwm_min_real);


  delay(200);
}

void find_angle_range() {
  RA_find_angle_range();
  //DEC_find_angle_range();
  //FOCUS_find_angle_range();
}
// ==========================
// PID DEC
// ==========================
void DEC_update_motor() {

  unsigned long now = millis();

  double dt = (now - DEC_prevTime) / 1000.0;
  if (dt <= 0) dt = 0.001;

  long pos = DEC_encoder_reading();

  long error_ticks = DEC_target_ticks - pos;

  double error_deg = error_ticks * 360.0 / TICKS_PER_REV / DEC_gearbox;

  double p_term = DEC_Kp * error_deg;

  DEC_integralError += error_deg * dt;

  if (DEC_integralError > 100) DEC_integralError = 100;
  if (DEC_integralError < -100) DEC_integralError = -100;

  double i_term = DEC_Ki * DEC_integralError;

  double d_error = (error_deg - DEC_prevErrorDeg) / dt;

  double d_term = DEC_Kd * d_error;

  double pwm_f = p_term + i_term + d_term;

  int pwm = (int)pwm_f;

  pwm = constrain(pwm, -pwm_max_driver, pwm_max_driver);

  if (pwm > 0 && pwm < DEC_pwm_min_real) {

    pwm = DEC_pwm_min_real;
    DEC_integralError = 0;
  }

  if (pwm < 0 && pwm > -DEC_pwm_min_real) {

    pwm = -DEC_pwm_min_real;
    DEC_integralError = 0;
  }

  DEC_drive_motor(pwm);

  DEC_prevErrorDeg = error_deg;
  DEC_prevTime = now;
}

// ==========================
// PID RA
// ==========================
void RA_update_motor() {

  unsigned long now = millis();

  double dt = (now - RA_prevTime) / 1000.0;

  if (dt <= 0) dt = 0.001;

  long pos = RA_encoder_reading();

  long error_ticks = RA_target_ticks - pos;

  double error_deg = error_ticks * 360.0 / TICKS_PER_REV / RA_gearbox;

  double p_term = RA_Kp * error_deg;

  RA_integralError += error_deg * dt;

  if (RA_integralError > 100) RA_integralError = 100;
  if (RA_integralError < -100) RA_integralError = -100;

  double i_term = RA_Ki * RA_integralError;

  double d_error = (error_deg - RA_prevErrorDeg) / dt;

  double d_term = RA_Kd * d_error;

  double pwm_f = p_term + i_term + d_term;

  int pwm = (int)pwm_f;

  pwm = constrain(pwm, -pwm_max_driver, pwm_max_driver);

  if (pwm > 0 && pwm < RA_pwm_min_real) {

    pwm = RA_pwm_min_real;
    RA_integralError = 0;
  }

  if (pwm < 0 && pwm > -RA_pwm_min_real) {

    pwm = -RA_pwm_min_real;
    RA_integralError = 0;
  }

  RA_drive_motor(pwm);

  RA_prevErrorDeg = error_deg;
  RA_prevTime = now;
}

// ==========================
// PID FOCUS
// ==========================

void FOCUS_update_motor() {

  unsigned long now = millis();

  double dt = (now - FOCUS_prevTime) / 1000.0;
  if (dt <= 0) dt = 0.001;

  long pos = FOCUS_encoder_reading();

  long error_ticks = FOCUS_target_ticks - pos;

  double error_deg = error_ticks * 360.0 / TICKS_PER_REV / FOCUS_gearbox;

  double p_term = FOCUS_Kp * error_deg;

  FOCUS_integralError += error_deg * dt;

  if (FOCUS_integralError > 100) FOCUS_integralError = 100;
  if (FOCUS_integralError < -100) FOCUS_integralError = -100;

  double i_term = FOCUS_Ki * FOCUS_integralError;

  double d_error = (error_deg - FOCUS_prevErrorDeg) / dt;

  double d_term = FOCUS_Kd * d_error;

  double pwm_f = p_term + i_term + d_term;

  int pwm = (int)pwm_f;

  pwm = constrain(pwm, -pwm_max_driver, pwm_max_driver);

  if (pwm > 0 && pwm < FOCUS_pwm_min_real) {
    pwm = FOCUS_pwm_min_real;
    FOCUS_integralError = 0;
  }

  if (pwm < 0 && pwm > -FOCUS_pwm_min_real) {
    pwm = -FOCUS_pwm_min_real;
    FOCUS_integralError = 0;
  }

  FOCUS_drive_motor(pwm);

  FOCUS_prevErrorDeg = error_deg;
  FOCUS_prevTime = now;
  //Serial.print("FOCUS error deg: "); Serial.print(error_deg); Serial.print("FOCUS error ticks: "); Serial.println(error_ticks);
}


// ==========================
// Update both motors
// ==========================
void update_motor() {

  DEC_update_motor();
  RA_update_motor();
  FOCUS_update_motor();
}


// ==========================
// Set target
// ==========================
void set_motor_dec_angle(double deg) {

  DEC_target_degree = deg;
  DEC_target_ticks = deg * DEC_gearbox * TICKS_PER_REV / 360.0;
  DEC_integralError = 0.0;
}

void set_motor_dec_ticks(long ticks) {

  DEC_target_ticks = ticks;
  DEC_target_degree = ticks * 360.0 / (DEC_gearbox * TICKS_PER_REV);
  DEC_integralError = 0.0;
}
void set_motor_ra_angle(double deg) {

  RA_target_degree = deg;
  RA_target_ticks = RA_target_degree * RA_gearbox * TICKS_PER_REV / 360.0;
  RA_integralError = 0.0;
}

void set_motor_focus_angle(double deg) {

  FOCUS_target_degree = deg;
  FOCUS_target_ticks = FOCUS_target_degree * FOCUS_gearbox * TICKS_PER_REV / 360.0;
  FOCUS_integralError = 0.0;
}


// ==========================
// Init motors
// ==========================
void motors_init() {
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

 pinMode(FOCUS_ENC_A, INPUT_PULLUP); // pin 50
  pinMode(FOCUS_ENC_B, INPUT_PULLUP); // pin 51
  FOCUS_prevState = (digitalRead(FOCUS_ENC_A) << 1) | digitalRead(FOCUS_ENC_B);
  last_portB = PINB;                   // initial state of port B for encoder and switches
  PCICR |= (1 << PCIE0);               // enable PCINT0
  PCMSK0 |= (1 << PCINT2) | (1 << PCINT3); // enable PB2/PB3
  FOCUS_encoderCount = 0;

  pinMode(FOCUS_DIR, OUTPUT);
  pinMode(FOCUS_PWM, OUTPUT);
}