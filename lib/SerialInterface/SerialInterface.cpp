#include "SerialInterface.h"
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


void print_curve_data(CurveParams *c)
{
  Serial.print("RR: ");
  Serial.print(c->rr,0);
  Serial.print(" X: ");
  Serial.print(c->x,0);
  Serial.print(" Tidal Vol: ");
  Serial.print(c->target_vol*1000, 0);
  Serial.print(" A_T: ");
  Serial.println(c->a_t, 0);
  Serial.print("Timming --> ");
  for(int i=0; i<7; i++)
  {
    Serial.print("T_");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(c->t[i]);
    if(i<=6) Serial.print(", ");
  }
  Serial.println();
  Serial.print("Velocities --> ");
  for(int i=0; i<7; i++)
  {
    Serial.print("V_");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(c->v[i], 4);
    if(i<=6) Serial.print(", ");
  }
  Serial.println();
  Serial.print("Plus C --> ");
  for(int i=0; i<7; i++)
  {
    Serial.print("C_");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(c->plus_c[i], 4);
    if(i<=6) Serial.print(", ");
  }
  Serial.println();
  Serial.print("Accels: ");
  Serial.print(c->accel[0]);
  Serial.print(", ");
  Serial.print(c->accel[1]);
  Serial.print(", ");
  Serial.println(c->accel[2]);
}

void plot_data(StepInfo *s, CurveParams *c, MotorDynamics *m, FlowData *f)
{
  #ifdef USE_FLUTTER_PRINTS
  static int16_t buffer_index = 0;
  static double graph_1[10];
  static double graph_2[10];
  static double graph_3[10];
  static uint32_t cycle_buf[10];
  static uint32_t steps[10];
  graph_1[buffer_index] = f->volume;
  graph_2[buffer_index] = f->flow*60;
  graph_3[buffer_index] = f->pressure;
  cycle_buf[buffer_index] = s->cycle;
  steps[buffer_index] = s->cur_step+(s->cycle%3*c->t_f);
  if (buffer_index++ == 9)
  {
    for(int i = 0; i<(sizeof(steps)/sizeof(steps[0])); i++)
    {
      Serial.print("DATATOGRAPH: ");
      Serial.print("cycle");
      Serial.print(cycle_buf[i]);
      Serial.print(",guno");
      Serial.print(graph_1[i], 5);
      Serial.print(",gdos");
      Serial.print(graph_2[i], 5);
      Serial.print(",gtres");
      Serial.print(graph_3[i], 5);
      Serial.print(",xxx");
      Serial.print(steps[i]);
      Serial.print(",");
      Serial.println();
    }
    Serial.print("DATATOGRAPH: ");
    Serial.print("cycle");
    Serial.print(cycle_buf[9]);
    Serial.print(",rUUuno");
    Serial.print(f->vti*1000,0);
    Serial.print(",rdos");
    Serial.print(f->vte*1000,0);
    Serial.print(",rtres");
    Serial.print(f->pip, 2);
    Serial.print(",rcuatro");
    Serial.print(f->peep, 2);
    Serial.print(",rcinco");
    Serial.print(f->flow_ins_max*60, 2);
    Serial.print(",rseis");
    Serial.print(f->flow_exp_max*60, 2);
    Serial.println(",");
    buffer_index = 0;
  }
  #else
  // Serial.print(pres_0.pressure_adc);
  // Serial.print(" ");
  // Serial.print(pres_0.openpressure);
  // Serial.print(" ");
  Serial.print(s->cur_step);
  Serial.print(" ");
  Serial.print(f->flow*60, 5);
  Serial.print(" ");
  Serial.print(f->volume, 5);
  Serial.print(" ");
  Serial.print(f->pressure, 5);
  Serial.print(" ");
  Serial.print((double)s->cur_stage/10, 4);
  Serial.print(" ");
  Serial.print(m->current_ang_pos, 4);
  Serial.print(" ");
  Serial.print(m->target_pos, 4);
  Serial.print(" ");
  Serial.print(m->current_vel, 4);
  Serial.print(" ");
  Serial.print(m->target_vel, 4);
  Serial.print(" ");
  Serial.print(c->t_f);
  Serial.print(" ");
  Serial.print(m->output);
  Serial.println();
  #endif
}

void parse_params(char buf[], unsigned long size, SysState *sys, ControlVals *con, CurveParams *c, CurveParams *n)
{
  char cmd[20];
  int32_t u_value_1 = 0;
  int32_t u_value_2 = 0;
  int32_t u_value_3 = 0;
  char temp_buf[100];
  memset(temp_buf, 0x00, sizeof(temp_buf)/sizeof(char));
  sscanf(buf, " %s %d %d %d ", cmd, &u_value_1, &u_value_2, &u_value_3);
  double value_1 = (double)u_value_1/10;
  double value_2 = (double)u_value_2/10;
  double value_3 = (double)u_value_3/10;

  if(!strcmp(cmd, "PING"))
  {
    Serial.println("PING");
  }
  if(!strcmp(cmd, "STOP"))
  {
    Serial.println("Homing and stoping the machine...");
    sys->play_state = STOP;
  }
  else if(!strcmp(cmd, "PAUSE"))
  {
    Serial.println("Pausing the machine...");
    sys->play_state = PAUSE;
  }
  else if(!strcmp(cmd, "RESUME"))
  {
    Serial.println("Resuming the machine...");
    sys->play_state = PLAY;
  }
  else if(!strcmp(cmd, "CAL"))
  {
    sys->cal_flag = 0;
  }
//   else if(!strcmp(cmd, "HCAL"))
//   {
//     cal_flag = 1;
//     h_cal_flag = 0;
//   }
  else if(!strcmp(cmd, "PLOT"))
  {
    sys->plot_enable = !sys->plot_enable;
  }
  else if(!strcmp(cmd, "PRINTP"))
  {
    print_curve_data(c);
  }
  else if(!strcmp(cmd, "PARAMS"))
  {
    if (value_1 == 0 || value_2 == 0 || value_3 == 0)
    {
      Serial.println("Not enough values!");
      Serial.print(value_1);
      Serial.print(value_2);
      Serial.print(value_3);
      return;
    }

    int param_check_var = params_check(value_1, value_2, vol_to_deg((double)u_value_3/1000));
    if(param_check_var == 0)
    {
      Serial.println("Combination accepted!");
      get_accels(n, value_1, value_2, vol_to_deg((double)u_value_3/1000));
      n->rr = value_1;
      n->x = value_2;
      n->a_t = vol_to_deg((double)u_value_3/1000);
      n->target_vol = (double)u_value_3/1000;
      sys->params_change_flag = 1;
    }
    else if (param_check_var == 1) Serial.println("Tidal volume too high! Ignoring...");
    else if (param_check_var == 2) Serial.println("Tidal volume too low! Ignoring...");
    else if (param_check_var == 3) Serial.println("Respiration rate too high! Ignoring...");
    else if (param_check_var == 4) Serial.println("Respiration rate too low! Ignoring...");
    else if (param_check_var == 5) Serial.println("Respiration relation too high! Ignoring...");
    else if (param_check_var == 6) Serial.println("Respiration ralation too low! Ignoring...");
    else Serial.println("Impossible combination! Ignoring...");
  }
  else if(!strcmp(cmd, "Kval"))
  {
    if (value_2 == 0 || value_3 == 0)
    {
      Serial.println("Not enough values!");
      Serial.print(value_1);
      Serial.print(value_2);
      Serial.print(value_3);
      return;
    }
    int k_check_var = k_check(value_1, value_2, value_3);
    if(k_check_var == 0)
    {
      con->kp[(uint16_t)value_1] = value_2;
      con->kv[(uint16_t)value_1] = value_3;
    }
    else if (k_check_var == 1) Serial.println("Index out of range! Ignoring...");
    else if (k_check_var == 2) Serial.println("No negative values allowed! Ignoring...");
    else if (k_check_var == 3) Serial.println("KP is too high! Ignoring...");
    else if (k_check_var == 4) Serial.println("KP is too low! Ignoring...");
    else if (k_check_var == 5) Serial.println("KV is too high! Ignoring...");
    else if (k_check_var == 6) Serial.println("KV is too low! Ignoring...");
    Serial.println(con->kp[(uint16_t)value_1]);
    Serial.println(con->kv[(uint16_t)value_1]);
  }

  else if(!strcmp(cmd, "Y0")) // This tunes the flow sensor to match testing equiptment
  {
    Serial.println(u_value_1);
    if (u_value_1 <= 0)
    {
      Serial.println("Value must be above 0.");
      return;
    }
    y_0 = (double)u_value_1 / 10000;
    Serial.println(y_0);
  }
  else Serial.println("CMD not recognized");
}