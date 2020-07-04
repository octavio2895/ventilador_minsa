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

void update_params(StepInfo *s, SysState *sys, CurveParams *c, CurveParams *n)
{
  static bool init;
  if (sys->params_change_flag && s->cur_step < 20)
  {
    c->a_t = n->a_t;
    c->rr = n->rr;
    c->x = n->x;
    c->target_vol = n->target_vol;
    c->t_f = 60000/c->rr;
    c->t_d = c->t_f/(c->x+1);
    c->accel[0] = n->accel[0];
    c->accel[1] = n->accel[1];
    c->accel[2] = n->accel[2];
    init = 0;
    sys->params_change_flag = 0;
  }
  if(!init)
  {
    double sq, den, plus;
    sq = sqrt(-1*(2*c->a_t*(c->accel[0]-c->accel[2]) + (c->accel[0]*c->accel[2]*((double)c->t_d/1000)*((double)c->t_d/1000)))*(c->accel[0]-c->accel[1])*(c->accel[1]-c->accel[2]));
    den = ((c->accel[0]-c->accel[2])*(c->accel[0]-c->accel[1]));
    plus = ((c->accel[0]- c->accel[1])*c->accel[2]*((double)c->t_d/1000));
    c->t[0] = (-1*(sq + plus) / den )*1000;
    c->v[0] = (c->accel[0] * ((double)c->t[0]/1000))*DEG_TO_RAD;
    c->t[1] = (((double)c->t[0]/1000) * ((c->accel[1]-c->accel[0])/(c->accel[1]-c->accel[2])) - (((double)c->t_d/1000)*((c->accel[2])/(c->accel[1]-c->accel[2]))))*1000;
    c->v[1] = (-c->accel[2]*(((double)c->t_d-(double)c->t[1])/1000))*DEG_TO_RAD;
    c->t[2] = c->t_d;
    c->v[2] = 0;
    c->v[3] = 0;
    c->t[3] = c->t[2] + BREATH_PAUSE;
    c->t[4] = c->t[3] + (sqrt((c->a_t)/(c->accel[0]))*1000);
    c->v[4] = -c->accel[0]*((double)(c->t[4]-c->t[3])/1000) * DEG_TO_RAD;
    c->v[5] = 0;
    c->t[5] = (2*c->t[4]) - c->t[3];
    c->t[6] = c->t_f;
    c->v[6] = 0;
    c->plus_c[0] = 0;
    c->plus_c[1] = 0.5*(c->v[0] * ((double)(c->t[0])/1000));
    c->plus_c[2] = c->plus_c[1] + (0.5*(double)(c->v[1] + c->v[0])*((double)(c->t[1] - c->t[0])/1000));
    c->plus_c[3] = c->plus_c[2] + (0.5*(double)(c->v[2] + c->v[1])*((double)(c->t[2] - c->t[1])/1000));
    c->plus_c[4] = c->plus_c[3] + (0.5*(double)(c->v[3] + c->v[2])*((double)(c->t[3] - c->t[2])/1000));
    c->plus_c[5] = c->plus_c[4] + (0.5*(double)(c->v[4] + c->v[3])*((double)(c->t[4] - c->t[3])/1000));
    c->plus_c[6] = c->plus_c[5] + (0.5*(double)(c->v[5] + c->v[4])*((double)(c->t[5] - c->t[4])/1000));
    // print_curve_data(c);
    init = 1;
  }
  // return;
}

void generate_curve(StepInfo *s, SysState *sys, MotorDynamics *m, CurveParams *c)
{
  if(sys->play_state == PLAY)
  {
    m->target_vel = get_target_velocity(s, c);
    m->target_pos = get_target_position(s, c);
  }
}

double get_target_position(StepInfo *s, CurveParams *c)
{
  double target_pos;
  if(s->cur_stage == INS_1)
  {
    target_pos = 0.5*(c->v[s->cur_stage]/((double)c->t[s->cur_stage]))*((double)(s->cur_step*s->cur_step/1000)) + c->plus_c[0];
    return target_pos;
  }
  else
  {
    target_pos = 0.5*((c->v[s->cur_stage] - c->v[s->cur_stage-1])/((double)(c->t[s->cur_stage] - c->t[s->cur_stage-1])/1000))*((double)(s->cur_step - c->t[s->cur_stage-1])/1000)*((double)(s->cur_step - c->t[s->cur_stage-1])/1000) + ((c->v[s->cur_stage-1])*((double)(s->cur_step - c->t[s->cur_stage-1])/1000)) + c->plus_c[s->cur_stage];
    return target_pos;
  }
  return target_pos;
}

double get_target_velocity(StepInfo *s, CurveParams *c)
{
  static double target_vel;
  if(s->cur_stage == INS_1)
  {
    target_vel = (c->v[s->cur_stage]/((double)c->t[s->cur_stage]))*((double)s->cur_step);
    return target_vel;
  }

  else if (s->cur_stage == REST_1 || s->cur_stage == REST_2)
  {
    target_vel = 0;
    return target_vel;
  }
  else
  {
    target_vel = c->v[s->cur_stage-1] + ((c->v[s->cur_stage] - c->v[s->cur_stage-1])/((double)(c->t[s->cur_stage] - c->t[s->cur_stage-1])))*((double)s->cur_step - (double)c->t[s->cur_stage-1]);
    return target_vel;
  }
  return target_vel;
}