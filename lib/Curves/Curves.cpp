#include "Curves.h"
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

int8_t params_check(double rr, double x, double a_t)
{
  if(a_t > MAX_VOL) return 1;
  if(a_t<MIN_VOL) return 2;
  if(rr>MAX_RR) return 3;
  if(rr<MIN_RR) return 4;
  if(x>MAX_X) return 5;
  if(x<MIN_X) return 6;
  double min_accel = rr*rr*a_t*(x+1)*(x+1)/600;
  double max_accel = 17*rr*rr*a_t*(x+1)*(x+1)/1800; // Actually, its just 5.666*min_accel. That probably depends on the constrain that m2 = -m1/16 and m3 = -m1/2.
  if(min_accel > MAX_ACCEL) return 7; // Cannot be run.
  else return 0; // Could be done by modifying acceleration.
  return -1; // Should never get here
}

void get_accels(CurveParams *c, double rr, double x, double a_t)
{
  double min_accel = rr*rr*a_t*(x+1)*(x+1)/600; // TODO: redoing this calculation, this should be optimized.
  double max_accel = 17*rr*rr*a_t*(x+1)*(x+1)/1800;
  c->accel[0] = max_accel > MAX_ACCEL ? (MAX_ACCEL + min_accel)/2 : (max_accel + min_accel)/2;
  c->accel[1] = -c->accel[0]/16;
  c->accel[2] = -c->accel[0]/2;
}