#ifndef MOTOR_CONSTANTS_HPP
#define MOTOR_CONSTANTS_HPP

////////////////////////////////////////////////////////////////
/////////// Global Motor constants
////////////////////////////////////////////////////////////////

#define TICKS_PER_REV 48
const int pwm_start_test = 1; // PWM minimal to start test
const int pwm_max_driver = 255;
const int repetitions = 3; // number of time to test minimum PWM
const int8_t quadTable[16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

////////////////////////////////////////////////////////////////
/////////// Pin definitions MOTOR DECLINATION (DEC)(BLUE)
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
/////////// Pin definitions MOTOR RIGHT ASCENSION (RA)(ORANGE)
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


#endif // MOTOR_CONSTANTS_HPP
