#ifndef MOTOR_CONSTANTS_HPP
#define MOTOR_CONSTANTS_HPP

////////////////////////////////////////////////////////////////
/////////// Global Motor constants
////////////////////////////////////////////////////////////////
#define TICKS_PER_REV 48
#define PWM_FREQ 20000
#define PWM_RES 10
const int pwm_start_test = 150; // PWM minimal to start test
const int pwm_max_driver = 1023;
const int8_t quadTable[16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

////////////////////////////////////////////////////////////////
/////////// Pin definitions MOTOR DECLINATION (DEC)
////////////////////////////////////////////////////////////////

// Encoder pins
#define DEC_ENC_A 34
#define DEC_ENC_B 35

// BTS7960 Motor Driver
#define DEC_RPWM 25
#define DEC_LPWM 26
#define DEC_REN  27
#define DEC_LEN  14
#define DEC_PWM_CH_R 0
#define DEC_PWM_CH_L 1

// Motor constants
const double DEC_gearbox = 900.0;
const double DEC_Kp = 100.0;   // coefficient P
const double DEC_Kd = 0.005;
////////////////////////////////////////////////////////////////
/////////// Pin definitions MOTOR RIGHT ASCENSION (RA)
////////////////////////////////////////////////////////////////
// Encoder pins
#define RA_ENC_A 18
#define RA_ENC_B 19

// BTS7960 Motor Driver
#define RA_RPWM 15
#define RA_LPWM 2
#define RA_REN  0
#define RA_LEN  4
#define RA_PWM_CH_R 0
#define RA_PWM_CH_L 1

// Motor constants
const double RA_gearbox = 500.0;
const double RA_Kp = 15.0;   // coefficient P
const double RA_Kd = 0.005;

#endif // MOTOR_CONSTANTS_HPP
