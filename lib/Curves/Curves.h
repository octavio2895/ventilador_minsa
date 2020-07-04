#ifndef CURVE_LIB
#define CURVE_LIB

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <SysStructs.h>

int8_t params_check(double, double, double);
void get_accels(CurveParams *, double, double, double);
void update_params(StepInfo *, SysState *, CurveParams *, CurveParams *);
void generate_curve(StepInfo *, SysState *, MotorDynamics *, CurveParams *);
double get_target_position(StepInfo *, CurveParams *);
double get_target_velocity(StepInfo *, CurveParams *);

#endif