#include "MotorController.h"
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


int8_t k_check(double stage, double kp, double kv)
{
  if(stage >= 3) return 1;
  if(stage < 0 || kp < 0 || kv < 0) return 2;
  if(kp > KP_MAX) return 3;
  if(kp < KP_MIN) return 4;
  if(kv > KV_MAX) return 5;
  if(kv < KV_MIN) return 6;
  else return 0;
}


int32_t calculate_position(ODriveArduino *o)
{
  int32_t pos = odrive_read_encoder(&Serial2, o, ARM_AXIS);
  return pos;
}

double calculate_angular_velocity(MotorDynamics *m)
{
  static double prev_angular_position;
  static uint32_t  old_millis;
  static double vels[5] = {0,0,0,0,0};
  static uint8_t stage = 0;
  vels[stage] = 1000*(m->current_ang_pos - prev_angular_position)/((double)(millis()- old_millis));
  if (++stage == 5) stage = 0;
  double sum = vels[0] + vels[1] + vels[2] + vels[3] + vels[4];
  double velocity = sum/5;
  prev_angular_position = m->current_ang_pos;
  old_millis = millis();
  return velocity;
}

double calculate_angular_velocity_fod3(MotorDynamics *m) // TODO: Review first order differential approximation
{
  static uint8_t index = 0, stage = 0;
  static double prev_angular_position, prev_velocity;
  double velocity;
  static uint32_t  old_millis;
  static double vels[5] = {0,0,0,0,0}, pos[3] = {0,0,0};
  if(++index >= 3) index = 0;
  pos[index] = m->current_ang_pos;
  double dh = 2*(millis() - old_millis)/1000;
  switch (index)
  {
    case 0:
      velocity = (-4*pos[2] + (pos[1]) + 3*pos[0])/dh;
      break;
    case 1:
      velocity = (-4*pos[0] + (pos[2]) + 3*pos[1])/dh;
      break;
    case 2:
      velocity = (-4*pos[1] + (pos[0]) + 3*pos[0])/dh;
      break;
  }
  vels[stage] = 1000*(m->current_ang_pos - prev_angular_position)/((double)(millis()- old_millis));
  if (++stage == 5) stage = 0;
  prev_angular_position = m->current_ang_pos;
  old_millis = millis();
  return velocity;
}

int16_t calculate_angular_acceleration(double ang_vel)
{
  static int16_t prev_velocity = 0;
  static uint32_t prev_millis = 0;
  int16_t acceleration = 1000*(ang_vel - prev_velocity)/(millis() - prev_millis);
  prev_velocity = ang_vel;
  prev_millis = millis();
  return acceleration;
}

void motor_write(MotorDynamics *m, double pwm)
{
  if(pwm > 0) digitalWrite(m->dir_pin, FORWARD_LOGIC);
  else digitalWrite(m->dir_pin, BACKWARD_LOGIC);
  analogWrite(m->pwm_pin, abs(pwm));
}

void odrive_speed_write(MotorDynamics *m, double vel)
{
  char serial_buf[100];
  #ifdef INVERTED_SPEED
  snprintf(serial_buf, sizeof(serial_buf),"v %d %ld", m->axis,(int32_t)-vel);
  #else
  snprintf(serial_buf, sizeof(serial_buf),"v %d %d", m->axis,(int32_t)vel);
  #endif
  m->serial_out->println(serial_buf);
}

void odrive_pos_write(MotorDynamics *m, double pos)
{
  char serial_buf[100];
  #ifdef INVERTED_SPEED
  snprintf(serial_buf, sizeof(serial_buf),"p %d %ld", m->axis,(int32_t)-pos);
  #else
  snprintf(serial_buf, sizeof(serial_buf),"p %d %d", m->axis,(int32_t)-pos);
  #endif
  m->serial_out->println(serial_buf);
}

void read_motor(MotorDynamics *m, ODriveArduino *o) //TODO Encapsulate ODrive into MotorDynamics
{
  m->current_pos = odrive_read_encoder(&Serial2, o, ARM_AXIS);
  // Serial.println(m->current_pos);
  m->current_ang_pos = CLICKS_TO_RAD * m->current_pos;
  m->current_vel = calculate_angular_velocity(m);
}

void filter_motor(MotorDynamics *m)
{
  static double prev_pwm;
  // m->output = (m->output + prev_pwm * FILTER_PWM) / (FILTER_PWM + 1); // TODO: Review this
  // prev_pwm = m->output;
  
  if(m->output > MAX_PWM)
  {
    m->output = MAX_PWM;
  }
  else if (m->output < -MAX_PWM)
  {
    m->output = -MAX_PWM;
  }

}

void execute_motor(SysState *sys, MotorDynamics *m)
{
  if (sys->play_state == PAUSE)
  {
    odrive_speed_write(m, 0);
    return;
  }
  else
  {
    odrive_pos_write(m, m->output+(sys->zero_pos));
  }
}

void mimo_control(MotorDynamics *m, ControlVals *c, StepInfo *s)
{
  static double error_position;
  static double error_velocity;
  static double motor_volts;
  double kp = 120000;//*m->current_ang_pos + 120;

  //Calculate error
  error_position = m->target_pos - m->current_ang_pos;
  error_velocity = m->target_vel - m->current_vel;
  // if(s->cur_stage >= REST_1) kp = K1;
  // kp = interpolate_gains(m->current_ang_pos);

  // m->motor_volts = kp * error_position;
  // m->output = fmap(m->motor_volts, 0, 12, MAX_PWM, -MAX_PWM);
  // m->output = kp * error_position;
  m->output = m->target_pos*RAD_TO_CLICK;
  // Serial.println(m->output);

}
double interpolate_gains(double ang)
{
  if(ang<K_ANG_1)
  {
    return(ang*((K_VAL_1-K_VAL_0/K_ANG_1-K_ANG_0))+K_VAL_0);
  }
  if(ang<K_ANG_2)
  {
    return(ang*((K_VAL_2-K_VAL_1/K_ANG_2-K_ANG_1))+K_VAL_1);
  }
  if(ang>=K_ANG_2)
  {
    return(ang*((K_VAL_3-K_VAL_2/K_ANG_3-K_ANG_2))+K_VAL_2);
  }
}

double fmap(double in, double in_min, double in_max, double out_min, double out_max)
{
  return((in/(in_max-in_min))*(out_max-out_min)+out_min);
}