#ifndef STEP_LIB
#define STEP_LIB

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <SysStructs.h>

void calc_step(StepInfo *s, SysState *sys, CurveParams *c);

#endif