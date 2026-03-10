#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

////////////////////////////////////////////////////////////////
/////////// Global Motor constants
////////////////////////////////////////////////////////////////
extern volatile uint8_t last_portB;
extern volatile uint8_t FOCUS_prevState;
extern volatile long FOCUS_encoderCount;

#define TICKS_PER_REV 48
const int pwm_start_test = 1; // PWM minimal to start test
const int pwm_max_driver = 255;
const int repetitions = 3; // number of time to test minimum PWM
const int8_t quadTable[16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

////////////////////////////////////////////////////////////////
/////////// (MOTOR 1) Pin definitions MOTOR DECLINATION(BLUE)
////////////////////////////////////////////////////////////////

// Encoder pins
#define DEC_ENC_A 2 //yellow
#define DEC_ENC_B 3 //brown

// BTS7960 Motor Driver
#define DEC_RPWM 6 //green
#define DEC_LPWM 7 //yellow
#define DEC_REN  22 //blue
#define DEC_LEN  23 //orange

// Motor constants
const int DEC_pwm_start_max = 150; // PWM maximal for the calibration test, if the minimum PWM is higher than this value, the test is redone
const double DEC_gearbox = 900.0;
const double DEC_Kp = 25.0;   // Proportional gain
const double DEC_Ki = 0.025;     // Integral gain
const double DEC_Kd = 0.001;   // Derivative gain

////////////////////////////////////////////////////////////////
/////////// (MOTOR 2) Pin definitions MOTOR RIGHT ASCENSION(ORANGE)
////////////////////////////////////////////////////////////////

// Encoder pins
#define RA_ENC_A 18 //yellow
#define RA_ENC_B 19 //brown

// BTS7960 Motor Driver
#define RA_RPWM 8 //green
#define RA_LPWM 9 //yellow
#define RA_REN  24 //blue
#define RA_LEN  25 //orange

// Motor constants
const int RA_pwm_start_max = 150; // PWM maximal for the calibration test, if the minimum PWM is higher than this value, the test is redone
const double RA_gearbox = 900.0;
const double RA_Kp = 25.0;    // Proportional gain
const double RA_Ki = 0.025;     // Integral gain
const double RA_Kd = 0.001;   // Derivative gain

////////////////////////////////////////////////////////////////
/////////// (MOTOR 3) Pin definitions MOTOR FOCUS (FOCUS)(GREEN)
////////////////////////////////////////////////////////////////

// Encoder pins (Green is GND and blue is VCC)
#define FOCUS_ENC_A 50 //yellow
#define FOCUS_ENC_B 51 //white
/*
// BTS7960 Motor Driver(not used for this version)
#define FOCUS_RPWM 10 //green
#define FOCUS_LPWM 11 //yellow
#define FOCUS_REN  26 //blue
#define FOCUS_LEN  27 //orange
*/
// in this version the driver for focus is a MD13S
#define FOCUS_DIR 10 //white
#define FOCUS_PWM 11 //yellow

// Motor constants
const int FOCUS_pwm_start_max = 150; // PWM maximal for the calibration test, if the minimum PWM is higher than this value, the test is redone
const double FOCUS_gearbox = 75.0;
const double FOCUS_Kp = 1;    // Proportional gain
const double FOCUS_Ki = 0.0;   // Integral gain
const double FOCUS_Kd = 0.00;   // Derivative gain

////////////////////////////////////////////////////////////////
/////////// (MOTOR 4) Pin definitions MOTOR NOT USED
////////////////////////////////////////////////////////////////

// Encoder pins
#define EXTRA_ENC_A 52 //yellow
#define EXTRA_ENC_B 53 //brown

// BTS7960 Motor Driver
#define EXTRA_RPWM 44 //green
#define EXTRA_LPWM 45 //yellow
#define EXTRA_REN  28 //blue
#define EXTRA_LEN  29 //orange

// Motor constants
const int EXTRA_pwm_start_max = 150; // PWM maximal for the calibration test, if the minimum PWM is higher than this value, the test is redone
const double EXTRA_gearbox = 900.0;
const double EXTRA_Kp = 25.0;    // Proportional gain
const double EXTRA_Ki = 0.025;     // Integral gain
const double EXTRA_Kd = 0.001;   // Derivative gain

////////////////////////////////////////////////////////////////
/////////// Pin definitions SWITCHES
////////////////////////////////////////////////////////////////
#define SWITCH_1 A8
extern volatile bool switch1_triggered;
#define SWITCH_2 A9
extern volatile bool switch2_triggered;
#define SWITCH_3 A10
extern volatile bool switch3_triggered;
#define SWITCH_4 A11
extern volatile bool switch4_triggered;
#define SWITCH_5 A12
extern volatile bool switch5_triggered;
#define SWITCH_6 A13
extern volatile bool switch6_triggered;
#define SWITCH_7 A14
extern volatile bool switch7_triggered;
#define SWITCH_8 A15
extern volatile bool switch8_triggered;
#define SWITCH_9  12
extern volatile bool switch9_triggered;
#define SWITCH_10 13
extern volatile bool switch10_triggered;


#endif // MOTORS_CONSTANTS_HPP
