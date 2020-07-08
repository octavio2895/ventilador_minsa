#include "FlowState.h"
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

double calculate_flow_venturi(PressureSensor *p)
{
  return (VENTURI_BIG_AREA*1000*(sqrt((2 * abs(p->pressure)) / (AIR_DENSITY * ((SQ_AREA_RATIO)-1)))));
}

void calculate_flow_oplate(FlowData *f)
{
  if(f->differential_pressure > 0) f->flow = sqrt(abs(f->differential_pressure)/y_0);
  else  f->flow = (-sqrt(abs(f->differential_pressure)/y_1));
}


void read_pressure(PressureSensor *p, FlowData *f)
{
  static int16_t adc[8];
  static int16_t i = 0;
  if(i>7)i = 0;
  adc[i] = analogRead(A0);
  p->pressure_adc = arr_average(adc, 8);
  p->pressure = ((p->pressure_adc - p->openpressure) * MULTIPLIER * 2.222);
  f->differential_pressure = p->pressure;
  i++;
}

void read_pressure_2(PressureSensor *p, FlowData *f)
{
  static int16_t adc[8];
  static int16_t i = 0;
  if(i>7)i = 0;
  adc[i] = abs(ads.readADC_Differential_2_3());
  p->pressure_adc = arr_average(adc, 8);
  if(p->pressure_adc <= (double)(p->openpressure+1)) p->pressure_adc = (double)p->openpressure; // TODO: Check this deadzone.
  p->pressure = ((p->pressure_adc - (double)p->openpressure) * MULTIPLIER_2 * 1.25* 10.1972*4.40); // cmH2O
  f->pressure = p->pressure;
  i++;
}


double arr_average(int16_t arr[], uint16_t size)
{
  double sum = 0;
  double average = 0;
  for(int i =  0; i<size; i++) 
  {
    sum += arr[i];
  }
  average = sum / ((double)size);
  return (average);
}

double arr_top(double arr[], uint16_t size)
{
  double top = 0;
  for(int i =  0; i<size; i++) 
  {
    if(arr[i] > top) top = arr[i];
  }
  return (top);
}

int16_t calibrate_pressure_sensor(PressureSensor *p) 
{
  int count = 0;
  uint32_t timer = 0;
  double open_pressure = 0;
  double average = 0;

  while (true)
  {
    if (timer < millis()) 
    {
      // Serial.println("Entering");
      if (p->id == 0)open_pressure += (double)analogRead(A0);
      else open_pressure += abs((double)ads.readADC_Differential_2_3());
      // Serial.println(open_pressure);
      count++;
      timer = millis() + 50;
    }
    if (count >= 15) 
    {
      p->openpressure = ((int16_t)open_pressure/(count));
      Serial.println(p->openpressure);
      Serial.println(open_pressure, 5);
      break;
    }
  }  
  return ((int16_t)average);
}

void calculate_flow_state(StepInfo *s, SysState *sys, RotaryEncoder *e, CurveParams *c,  FlowData *f) // TODO: Change to state machine, refactor the shit out of this
{
  static Stages prev_stage = INS_1;
  static double prev_flow = 0;
  static double init_angle = 0;
  static uint32_t prev_millis = millis();
  static int16_t open_pressure_adc[16];
  static int16_t exp_press[16];
  static uint16_t open_pressure_index = 0;
  static double top_pres[64];
  static uint16_t top_pres_index = 0;
  static double max_inspiration_flow = 0;
  static double max_expiration_flow = 0;

  if(prev_stage != INS_1 && s->cur_stage == INS_1)
  {
    f->vte = f->vti - f->volume;
    f->volume = 0; //Resets volume to avoid drifting.
    f->flow_exp_max = max_expiration_flow;
    #ifdef USE_FLUTTER_PRINTS
    Serial.print("glen");
    Serial.print(c->t_f*3);
    Serial.println(",");
    #else
    Serial.print(f->angle, 5);
    Serial.print(" ");
    Serial.print(f->flow_exp_max*60, 5);
    Serial.print(" ");
    Serial.print(f->flow_ins_max*60, 5);
    Serial.print(" ");
    Serial.print(f->vti, 5);
    Serial.print(" ");
    Serial.print(f->pip, 5);
    Serial.print(" ");
    Serial.println(f->vte, 5);
    #endif
  }
  f->volume += (f->flow*(millis() - prev_millis)/1000);
  if(prev_stage <= INS_3 && s->cur_stage >= REST_1)
  {
    f->angle = CLICKS_TO_RAD*RAD_TO_DEG*((e->getPosition()<<1) - init_angle);
    f->pip = arr_top(top_pres, 64);
    memset(top_pres, 0, sizeof(top_pres));
    f->vti = f->volume;
    f->flow_ins_max = max_inspiration_flow;
  }

  else if(prev_stage == REST_2 && s->cur_stage == INS_1) 
  {
    max_inspiration_flow = 0;
    f->peep = arr_average(exp_press, 16);
    init_angle = (e->getPosition()<<1);
  }

  else if(s->cur_stage >= INS_1 && s->cur_stage <= INS_3) 
  {
    if(f->flow > max_inspiration_flow) max_inspiration_flow = f->flow;
  }

  if(s->cur_stage == INS_3)
  {
    if(top_pres_index > 63) top_pres_index = 0;
    top_pres[top_pres_index++] = f->pressure;
  }

  if(s->cur_stage >= INS_3 && s->cur_stage <= REST_2)
  {
    if(f->flow < max_expiration_flow) max_expiration_flow = f->flow;
  }

  if(s->cur_stage == REST_2)
  {
    if(open_pressure_index > 7) open_pressure_index = 0;
    exp_press[open_pressure_index] = (int16_t)f->pressure;
    // open_pressure_adc[open_pressure_index++] = (int16_t)pres_0.pressure_adc;
  }
  prev_millis = millis();
  prev_stage = s->cur_stage;
  prev_flow = f->flow; // TODO: Reimplement trapezoidal approximation
  
  if (sys->play_state==PAUSE) f->volume = 0;
}

void flow_controller(StepInfo *s, SysState *sys, ControlVals *con, CurveParams *c, CurveParams *n, FlowData *f)
{
  static int16_t prev_stage = 0;
  if(prev_stage <= INS_3 && s->cur_stage >= REST_1)
  {
    if(f->vti > c->target_vol*(1.1) || f->vti < c->target_vol*(.9))
    {
      double error = f->vti - c->target_vol;
      double new_ang = error > 0? c->a_t - abs(vol_to_deg(error*con->kvol)) : c->a_t + abs(vol_to_deg(error*con->kvol));
      if(new_ang > MAX_VOL) new_ang = MAX_VOL;
      if(new_ang != c->a_t && params_check(c->rr, c->x, new_ang) == 0)
      {
        get_accels(n, c->rr, c->x, new_ang);
        sys->params_change_flag = 1;
        n->a_t = new_ang;
        n->target_vol = c->target_vol;
        n->x = c->x;
        n->rr = c->rr;
      }
    }
  }
  prev_stage = s->cur_stage;
}

double deg_to_vol(double deg)
{
  return((22.1944588*deg-218.41009)/1000);
}

double vol_to_deg(double vol)
{
  return(44.8549*vol+9.93772);
}