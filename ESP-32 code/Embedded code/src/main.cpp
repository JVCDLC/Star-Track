#include <Arduino.h>
#include "motor_constants.hpp"

int DEC_pwm_min_real = 0; // PWM minimal required to move the motor
volatile long DEC_encoderCount = 0;
volatile uint8_t DEC_prevState = 0;

// ==========================
// Control variable
// ==========================
double DEC_target_degree = 0; // modified via motor_angle()
long DEC_target_ticks = 0;
double DEC_prevErrorDeg = 0.0;
unsigned long DEC_prevTime = 0;


void drive_motor(int pwm) { // Send PWM to the motor
  pwm = constrain(pwm, -pwm_max_driver, pwm_max_driver);
  if(pwm > 0){ ledcWrite(DEC_PWM_CH_R, pwm); ledcWrite(DEC_PWM_CH_L,0);}
  else if(pwm < 0){ ledcWrite(DEC_PWM_CH_R,0); ledcWrite(DEC_PWM_CH_L,-pwm);}
  else {ledcWrite(DEC_PWM_CH_R,0); ledcWrite(DEC_PWM_CH_L,0);}
}
// ==========================
// Interuption handler for encoder
// ==========================
void IRAM_ATTR ISR_Encoder() {
  uint8_t a = digitalRead(DEC_ENC_A);
  uint8_t b = digitalRead(DEC_ENC_B);
  uint8_t state = (a << 1) | b;
  uint8_t index = (DEC_prevState << 2) | state;
  DEC_encoderCount += quadTable[index];
  DEC_prevState = state;
}
// ==========================
// Read encoder value and make sure no interrupt create problem
// ==========================
long encoder_reading() {
  noInterrupts();
  long pos = DEC_encoderCount;
  interrupts();
  return pos;
}

// ==========================
// Find minimum PWM to rotate motor
// ==========================
int findMinimumPWMOnce(bool positive=true) {
  int step = 5;             
  int pwmTest = pwm_start_test - step; 
  long startPos = encoder_reading();
  
  while(pwmTest <= pwm_max_driver) {
    pwmTest += step;
    int pwm = positive ? pwmTest : -pwmTest;
    drive_motor(pwm);
    delay(200); // give time for the motor to react
    long delta = abs(encoder_reading() - startPos);
    if(delta > 1) { // Check if the motor turned
      drive_motor(0);
      delay(200);
      return pwmTest; 
    }
  }
  drive_motor(0);
  delay(200);
  Serial.print("value to high of pwm  to rotate the motor");
  return pwm_max_driver; // return max if nothing as been detected
}
// ==========================
// Find minimum PWM three time and use the max value  
// ==========================
int findMinimumPWM(int repetitions = 3) {
  int pwm_max=0;
  for(int i=0; i<repetitions; i++) {
    int pwm = 0;
    while(pwm < pwm_start_test) {
      Serial.print("Test "); Serial.print(i+1); Serial.println(" off PWM minimal...");
      pwm = findMinimumPWMOnce(true);
      if(pwm < pwm_start_test) {
        Serial.println("PWM too low, retest...");
        delay(500);
      }
    }
    if(pwm>pwm_max){
      pwm_max=pwm;
    }
    Serial.print("PWM detecte: "); Serial.println(pwm);
    delay(200);

  }
  return pwm_max;
}

// ==========================
// FONCTIONS DE CONTROLE
// ==========================
void motor_angle(double deg) {
  DEC_target_degree = deg;
  DEC_target_ticks = DEC_target_degree * DEC_gearbox * TICKS_PER_REV / 360.0;
}
void update_motor() {
  unsigned long now = millis();
  double dt = (now - DEC_prevTime) / 1000.0; // secondes
  if (dt <= 0) dt = 0.001; // sécurité

  long pos = encoder_reading();
  long errorTicks = DEC_target_ticks - pos;
  double errorDeg = errorTicks * 360.0 / TICKS_PER_REV / DEC_gearbox;

  // ----- D term -----
  double dError = (errorDeg - DEC_prevErrorDeg) / dt;

  // ----- PD control -----
  double pwm_f = DEC_Kp * errorDeg + DEC_Kd * dError;
  int pwm = (int)pwm_f;

  // ----- PWM limits -----
  pwm = constrain(pwm, -pwm_max_driver, pwm_max_driver);

  // Minimal PWM to overcome friction
  if (pwm > 0 && pwm < DEC_pwm_min_real) pwm = DEC_pwm_min_real;
  if (pwm < 0 && pwm > -DEC_pwm_min_real) pwm = -DEC_pwm_min_real;

  drive_motor(pwm);

  // ----- Save state -----
  DEC_prevErrorDeg = errorDeg;
  DEC_prevTime = now;

  // ----- Debug (optionnel) -----
  Serial.print("Err(deg): "); Serial.print(errorDeg, 3);
  Serial.print(" | dErr: "); Serial.print(dError, 3);
  Serial.print(" | PWM: "); Serial.println(pwm);
}


// ==========================
// SETUP
// ==========================
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(DEC_ENC_A, INPUT_PULLUP);
  pinMode(DEC_ENC_B, INPUT_PULLUP);
  DEC_prevState = (digitalRead(DEC_ENC_A) << 1) | digitalRead(DEC_ENC_B);
  attachInterrupt(digitalPinToInterrupt(DEC_ENC_A), ISR_Encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DEC_ENC_B), ISR_Encoder, CHANGE);
  DEC_encoderCount = 0;

  pinMode(DEC_REN, OUTPUT);
  pinMode(DEC_LEN, OUTPUT);
  digitalWrite(DEC_REN,HIGH);
  digitalWrite(DEC_LEN,HIGH);

  ledcSetup(DEC_PWM_CH_R,PWM_FREQ,PWM_RES);
  ledcSetup(DEC_PWM_CH_L,PWM_FREQ,PWM_RES);
  ledcAttachPin(DEC_RPWM,DEC_PWM_CH_R);
  ledcAttachPin(DEC_LPWM,DEC_PWM_CH_L);

  Serial.println("Systeme pret - detection automatique du PWM minimal");
  DEC_pwm_min_real = findMinimumPWM(3);
  Serial.print("PWM minimal initialise: "); Serial.println(DEC_pwm_min_real);

  // Initial position
  motor_angle(0); 
}

// ==========================
// LOOP
// ==========================
void loop() {
  update_motor();
  delay(20); // small delay to not saturate cpu
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
      motor_angle(value.toFloat());
      Serial.print("ACK REQUEST | Motor: ");
      Serial.print(motor);
      Serial.print(" | Target: ");
      Serial.println(value.toFloat(), 3);
    } else {
      Serial.println("ERR: not a valid command");
    }
  }
}
