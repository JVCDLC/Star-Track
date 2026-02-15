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
const int repetitions = 3; // number of time to test minimum PWM
const int8_t quadTable[16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

////////////////////////////////////////////////////////////////
/////////// Pin definitions MOTOR DECLINATION (DEC)(BLUE)
////////////////////////////////////////////////////////////////

// Encoder pins
#define DEC_ENC_A 34 //yellow
#define DEC_ENC_B 35 //brown

// BTS7960 Motor Driver
#define DEC_RPWM 25 //green
#define DEC_LPWM 32 //yellow
#define DEC_REN  27 //blue
#define DEC_LEN  33 //orange
#define DEC_PWM_CH_R 0
#define DEC_PWM_CH_L 1

// Motor constants
const int DEC_pwm_start_max = 400; // PWM maximal for the calibration test, if the minimum PWM is higher than this value, the test is redone
const double DEC_gearbox = 900.0;
const double DEC_Kp = 100.0;   // Proportional gain
const double DEC_Ki = 0.1;     // Integral gain
const double DEC_Kd = 0.005;   // Derivative gain

////////////////////////////////////////////////////////////////
/////////// Pin definitions MOTOR RIGHT ASCENSION (RA)(ORANGE)
////////////////////////////////////////////////////////////////

// Encoder pins
#define RA_ENC_A 23 //yellow
#define RA_ENC_B 22 //brown

// BTS7960 Motor Driver
#define RA_RPWM 17 //green
#define RA_LPWM 18 //yellow
#define RA_REN  19 //blue
#define RA_LEN  21 //orange
#define RA_PWM_CH_R 2
#define RA_PWM_CH_L 3

// Motor constants
const int RA_pwm_start_max = 600; // PWM maximal for the calibration test, if the minimum PWM is higher than this value, the test is redone
const double RA_gearbox = 900.0;
const double RA_Kp = 100.0;    // Proportional gain
const double RA_Ki = 0.1;     // Integral gain
const double RA_Kd = 0.005;   // Derivative gain


#endif // MOTOR_CONSTANTS_HPP
