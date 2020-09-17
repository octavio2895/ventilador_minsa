#ifndef SERIAL_LIB
#define SERIAL_LIB

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Curves.h>
#include <StepData.h>
#include <MotorController.h>
#include <FlowState.h>
#include <SysState.h>
#include <SysStructs.h>


void print_curve_data(CurveParams *);
void plot_data(StepInfo *, CurveParams *, MotorDynamics *, FlowData *);
void parse_params(char*, unsigned long, SysState*, ControlVals*, CurveParams*, CurveParams*);

#endif