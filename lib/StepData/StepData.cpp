#include "StepData.h"
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

void calc_step(StepInfo *s, SysState *sys, CurveParams *c)
{
  static bool init = 0, was_paused = 0;
  if (sys->restart_step_flag)
  {
    init = 0;
    s->cur_stage = INS_1;
    s->cur_step = 0;
    s->cur_millis = 0;
    sys->restart_step_flag = 0;
  }
  if (!init)
  {
    s->start_millis = millis();
    init = true;
    sys->restart_step_flag = 0;
  }
  if(was_paused && sys->play_state != PAUSE)
  {
    s->start_millis = millis() - s->cur_step;
    was_paused = false;
  }
  if (sys->play_state != PAUSE)
  {
    if(millis()-s->start_millis >=  c->t_f) 
    {
      s->start_millis = millis();
      s->cycle++;
    }
    s->cur_step = (millis()-s->start_millis);
    if(s->cur_step <= c->t[0])s->cur_stage = INS_1;
    else if(s->cur_step <= c->t[1])s->cur_stage = INS_2;
    else if(s->cur_step <= c->t[2])s->cur_stage = INS_3;
    else if(s->cur_step <= c->t[3])s->cur_stage = REST_1;
    else if(s->cur_step <= c->t[4])s->cur_stage = EXP_1;
    else if(s->cur_step <= c->t[5])s->cur_stage = EXP_2;
    else s->cur_stage = REST_2;
  }
  else
  {
    if(!was_paused) was_paused = true;
    return;
  }
}