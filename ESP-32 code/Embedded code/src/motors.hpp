#ifndef MOTORS_HPP
#define MOTORS_HPP
#include <Arduino.h>

extern int DEC_pwm_min_real;
extern int RA_pwm_min_real;

extern unsigned long DEC_prevTime;
extern unsigned long RA_prevTime;

void motors_init();

void update_motor();

void set_motor_dec_angle(double deg);
void set_motor_ra_angle(double deg);

void calibrate_minimum_PWM();
#endif // MOTOR_CONSTANTS_HPP