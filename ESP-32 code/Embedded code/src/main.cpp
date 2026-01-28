#include <Arduino.h>
#include <Arduino.h>

// ==========================
// ENCODER
// ==========================
#define ENC_A 34
#define ENC_B 35
#define TICKS_PER_REV 48
volatile long encoderCount = 0;
volatile uint8_t prevState = 0;
const int8_t quadTable[16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

// ==========================
// BTS7960 Motor Driver
// ==========================
#define RPWM 25
#define LPWM 26
#define REN  27
#define LEN  14
#define PWM_CH_R 0
#define PWM_CH_L 1
#define PWM_FREQ 20000
#define PWM_RES 10

const int pwm_max_driver = 1023;
int pwm_min_real = 0; // PWM minimal required to move the motor
const int pwm_start_test = 150; // PWM minimal to start test

// ==========================
// Control variable
// ==========================
double gearbox = 50.0;
double Kp = 12.0;   // coefficient P
double target_degree = 0; // modified via motor_angle()
long target_ticks = 0;
float angle_motor=0;


void drive_motor(int pwm) { // Send PWM to the motor
  pwm = constrain(pwm, -pwm_max_driver, pwm_max_driver);
  if(pwm > 0){ ledcWrite(PWM_CH_R, pwm); ledcWrite(PWM_CH_L,0);}
  else if(pwm < 0){ ledcWrite(PWM_CH_R,0); ledcWrite(PWM_CH_L,-pwm);}
  else {ledcWrite(PWM_CH_R,0); ledcWrite(PWM_CH_L,0);}
}
// ==========================
// Interuption handler for encoder
// ==========================
void IRAM_ATTR ISR_Encoder() {
  uint8_t a = digitalRead(ENC_A);
  uint8_t b = digitalRead(ENC_B);
  uint8_t state = (a << 1) | b;
  uint8_t index = (prevState << 2) | state;
  encoderCount += quadTable[index];
  prevState = state;
}
// ==========================
// Read encoder value and make sure no interrupt create problem
// ==========================
long encoder_reading() {
  noInterrupts();
  long pos = encoderCount;
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
  target_degree = deg;
  target_ticks = target_degree * gearbox * TICKS_PER_REV / 360.0;
}

// Appelée très souvent dans loop()
void update_motor() {
  long pos = encoder_reading();
  long errorTicks = target_ticks - pos;
  double errorDeg = errorTicks * 360.0 / TICKS_PER_REV / gearbox;

  int pwm = Kp * errorDeg;

  // If pwm too low, force it to be minimal pwm
  if(pwm > 0 && pwm < pwm_min_real) pwm = pwm_min_real;
  if(pwm < 0 && pwm > -pwm_min_real) pwm = -pwm_min_real;

  // Deadzone 
  //if(abs(errorTicks) <= 1) pwm = 0; // ±1 tick

  drive_motor(pwm);

  // Display values
  Serial.print("Pos ticks: "); Serial.print(pos);
  Serial.print(" | Target ticks: "); Serial.print(target_ticks);
  Serial.print(" | Err ticks: "); Serial.print(errorTicks);
  Serial.print(" | Pos deg: "); Serial.print(pos*360.0/TICKS_PER_REV/gearbox,2);
  Serial.print(" | Err deg: "); Serial.print(errorDeg,2);
  Serial.print(" | PWM: "); Serial.println(pwm);
}

// ==========================
// SETUP
// ==========================
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  prevState = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  attachInterrupt(digitalPinToInterrupt(ENC_A), ISR_Encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), ISR_Encoder, CHANGE);
  encoderCount = 0;

  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);
  digitalWrite(REN,HIGH);
  digitalWrite(LEN,HIGH);

  ledcSetup(PWM_CH_R,PWM_FREQ,PWM_RES);
  ledcSetup(PWM_CH_L,PWM_FREQ,PWM_RES);
  ledcAttachPin(RPWM,PWM_CH_R);
  ledcAttachPin(LPWM,PWM_CH_L);

  Serial.println("Systeme pret - detection automatique du PWM minimal");
  pwm_min_real = findMinimumPWM(3);
  Serial.print("PWM minimal initialise: "); Serial.println(pwm_min_real);

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



