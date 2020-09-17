#ifndef MOTOR_LIB
#define MOTOR_LIB

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Encoder.h>
#include <SysStructs.h>
#include <ODriveArduino.h>

#define KP_MAX                  300
#define KP_MIN                  0
#define KV_MAX                  50
#define KV_MIN                  0
#define FORWARD_LOGIC           1
#define BACKWARD_LOGIC          0
#define K_ANG_0                 0
#define K_VAL_0                 120
#define K_ANG_1                 0.4
#define K_VAL_1                 200
#define K_ANG_2                 0.6
#define K_VAL_2                 250
#define K_ANG_3                 0.7
#define K_VAL_3                 500
//Experiment
#define FILTER                0
#define FILTER_PWM            0
#define MAX_PWM               256


int8_t k_check(double, double, double);
double fmap(double in, double in_min, double in_max, double out_min, double out_max);
void mimo_control(MotorDynamics *m, ControlVals *c, StepInfo *s);
void execute_motor(SysState *sys, MotorDynamics *m);
void filter_motor(MotorDynamics *m);
void read_motor(MotorDynamics *m, RotaryEncoder *e);
void motor_write(MotorDynamics *m, double);
int16_t calculate_angular_acceleration(double ang_vel);
double calculate_angular_velocity_fod3(MotorDynamics *m);
double calculate_angular_velocity(MotorDynamics *m);
int16_t calculate_position(RotaryEncoder *e);
double interpolate_gains(double);
void odrive_speed_write(MotorDynamics *m, double vel);

#endif