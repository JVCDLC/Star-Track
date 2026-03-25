#ifndef MOTORS_HPP
#define MOTORS_HPP
#include <Arduino.h>
#include "constants.hpp"

extern int DEC_pwm_min_real;
extern int RA_pwm_min_real;
extern int FOCUS_pwm_min_real;

extern unsigned long DEC_prevTime;
extern unsigned long RA_prevTime;
extern unsigned long FOCUS_prevTime;

extern long DEC_min_angle;
extern long DEC_max_angle;
extern long RA_min_angle;
extern long RA_max_angle;
extern long FOCUS_min_angle;
extern long FOCUS_max_angle;

void motors_init();

void update_motor();
void DEC_drive_motor(int pwm);
void RA_drive_motor(int pwm);
void FOC_drive_motor(int pwm);

long DEC_encoder_reading();
long RA_encoder_reading();
long FOC_encoder_reading();

void calibrate_minimum_PWM();

int DEC_find_minimum_PWM();
int RA_find_minimum_PWM();
int FOC_find_minimum_PWM();

void find_angle_range();

void RA_find_angle_range();
void DEC_find_angle_range();
void FOC_find_angle_range();

void DEC_update_motor();
void RA_update_motor();
void FOC_update_motor();

void set_motor_DEC_angle(double deg);
void set_motor_DEC_ticks(long ticks);
void set_motor_RA_angle(double deg);
void set_motor_RA_ticks(long ticks);
void set_motor_FOC_angle(double deg);
void set_motor_FOC_ticks(long ticks) ;


#endif // MOTOR_CONSTANTS_HPP