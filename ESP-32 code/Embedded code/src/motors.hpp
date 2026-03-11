#ifndef MOTORS_HPP
#define MOTORS_HPP
#include <Arduino.h>

extern int DEC_pwm_min_real;
extern int RA_pwm_min_real;
extern int FOCUS_pwm_min_real;

extern unsigned long DEC_prevTime;
extern unsigned long RA_prevTime;
extern unsigned long FOCUS_prevTime;

void motors_init();

void update_motor();
void DEC_drive_motor(int pwm);
void RA_drive_motor(int pwm);
void FOCUS_drive_motor(int pwm);

long DEC_encoder_reading();
long RA_encoder_reading();
long FOCUS_encoder_reading();

void calibrate_minimum_PWM();

int DEC_find_minimum_PWM();
int RA_find_minimum_PWM();
int FOCUS_find_minimum_PWM();

void DEC_update_motor();
void RA_update_motor();
void FOCUS_update_motor();

void set_motor_dec_angle(double deg);
void set_motor_ra_angle(double deg);
void set_motor_focus_angle(double deg);


#endif // MOTOR_CONSTANTS_HPP