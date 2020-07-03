#ifndef CURVE_LIB
#define CURVE_LIB

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define K1                    37.5//85
#define K2                    1.25//4
#define K3                    120
#define K4                    0//4
#define K5                    150//4
#define K6                    0//4
#define ACCEL_1               320
#define ACCEL_2               -20
#define ACCEL_3               -160
#define MAX_ACCEL             700
#define MAX_VOL               55
#define MIN_VOL               0
#define MAX_RR                40
#define MIN_RR                0
#define MAX_X                 4
#define MIN_X                 1

struct CurveParams
{
  double rr = 15;
  double x = 1;
  uint32_t t_f = 60000/rr;
  uint32_t t_d = t_f/(x+1);
  double kv[3] = {K2, K4, K6};
  double kp[3] = {K1, K3, K5};

  double accel[3] = 
  {
    ACCEL_1,
    ACCEL_2,
    ACCEL_3
  };
  double v[7];
  uint32_t t[7];
  double plus_c[7];
  double target_vol = .7;
  double a_t = 42;

};

int8_t params_check(double, double, double);
void get_accels(CurveParams *, double, double, double);