#include <Arduino.h>
#include <RotaryEncoder.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <math.h>


// Pin definitions.
#define PIN_ENCODER_A PA7
#define PIN_ENCODER_B PA6
#define PIN_DIR_A PA4
#define PIN_DIR_B PA5
#define PIN_PWM PA3
#define PIN_LIMIT_SWITCH PA0
#define PIN_INVERT PB10
#define PIN_OPEN PA0 // UPDATE kp de 0.01 - 0.25
#define PIN_REGULADOR_RANGO PA1 //UPDATE ANGULO DE OPERACION
#define PIN_REGULADOR_TIEMPO PA2 //

// Physical constraints.
#define ROTARYMIN 0
#define ROTARYMAX 200
#define ROTARYINITIAL 0
#define ENCODER_CPR 2000
#define MOTOR_DZ 0 //165
#define BACKLASH_CLICKS 5
#define PROTECTION_CLICKS 5
#define ACCEL_1 16
#define ACCEL_2 -1
#define ACCEL_3 -8
#define ACCEL_4 0.23

// Update rates.
#define MOTOR_UPDATE_DELAY    10
#define MOTOR_SPEED_DELAY     20
#define SCREEN_UPDATE_DELAY   100
#define BLINK_DELAY           500
#define TARGET_UPDATE_DELAY   10
#define PARAMS_UPDATE_DELAY   100

// Parameter
#define CAL_PWM 10
#define CAL_DRIVE_BACK_ANG 5
#define CAL_DRIVE_ANG 0.5
#define CAL_DZ 0.5

//Super Calibrate
#define CAL_TICKS 5

//Experiment
#define RISE_TIME     3000
#define DEAD_TIME     0//1000
#define FALL_TIME     2000
#define WAITING_TIME  2000
#define ACC_TIME      800
#define DEGREES       360
#define FILTER        0
#define FILTER_PWM    0

#define K1            85
#define K2            4
#define K3            150
#define K4            4
#define K1_W          35
#define K2_W          1
//Targets times
#define FIRST_TIME         600
#define SECOND_TIME        2000
#define THIRD_TIME         3000

//Target degrees
#define F_DEG         5
#define S_DEG         11
#define T_DEG         15

//Target deg/s
#define DEG_P_SEG     8
#define DEG_TO_RAD    PI/180

//Control encoder
struct ControlVals 
{
  double kp;
  double kv;
}control_vals;

struct LcdVals
{
  int16_t encoder_pos;
}lcd_vals;

struct MotorDynamics
{
  int16_t lower_limit = 0;
  int16_t upper_limit;
  int16_t current_pos = 0;
  double current_ang_pos = 0;
  double target_pos;
  double current_vel;
  double target_vel;
  uint16_t output_range = 256;
  int16_t output;
}motor_vals;

struct CurveParams
{
  double const_vel_time_rise = RISE_TIME - 2*ACC_TIME;
  double const_vel_time_fall = FALL_TIME - 2*ACC_TIME;
  double theta_dot_max_rise = DEGREES/(const_vel_time_rise+ACC_TIME);
  double theta_dot_max_fall = 36/10;
  uint32_t t_d = 2000;
  double a_t = 10;
  uint32_t t_f = 6000;

  double accel[3] = 
  {
    ACCEL_1,
    ACCEL_2,
    ACCEL_3
  };

  // double v[7] = 
  // {
  //   0.14428637,
  //   0.13488079,
  //   0,
  //   0,
  //   -0.14428637,
  //   0,
  //   0
  // };

  // uint32_t t[7] = 
  // {
  //   515,
  //   1034,
  //   2000,
  //   3000,
  //   4117,
  //   5234,
  //   6000
  // };

  double v[7];
  uint32_t t[7];
  double plus_c[7];

  // double plus_c[7] =
  // {
  //   0,
  //   0.5*(v[0] * ((double)(t[0])/1000)),
  //   plus_c[1] + (0.5*(double)(v[1] + v[0])*((double)(t[1] - t[0])/1000)),
  //   plus_c[2] + (0.5*(double)(v[2] + v[1])*((double)(t[2] - t[1])/1000)),
  //   plus_c[3] + (0.5*(double)(v[3] + v[2])*((double)(t[3] - t[2])/1000)),
  //   plus_c[4] + (0.5*(double)(v[4] + v[3])*((double)(t[4] - t[3])/1000)),
  //   plus_c[5] + (0.5*(double)(v[5] + v[4])*((double)(t[5] - t[4])/1000))
  // };

}curve_vals, new_vals;

// Global vars.

bool cal_flag = false, enc_inverted = false, dir, is_rising=1, is_falling=0;
uint16_t zero_position = 0;
uint32_t next_motor_update = 0, next_speed_update = 0, next_dir_change, next_screen_update, next_params_update;
double pos_to_vel;

// Global objects.
RotaryEncoder encoder(PIN_ENCODER_A, PIN_ENCODER_B, PB0);
LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);  // Set the LCD I2C address
Servo myservo;

// Prototypes
void change_control_values(ControlVals *vals);
void BlinkLED();
void calibrate();
void motor_set_dir(double);
void motor_write(double);
int16_t calculate_position();
double calculate_angular_velocity(double);
int16_t calculate_angular_acceleration(double);
void encoderISR();
int16_t deg_to_clicks(double deg);
double clicks_to_rad(int16_t clicks);
float arr_average(float *arr, uint16_t size);
void mimo_control(MotorDynamics*, ControlVals*);
double get_target_position(uint32_t, CurveParams*);
double get_target_velocity(uint32_t, CurveParams*);
double deg_to_rad(double);
void pulse_interrupt();
void lcd_update(LcdVals *lcd_vals);
bool backlash_protection(double);
bool blocked_motor_protection(double, double);
void generate_curve(MotorDynamics*, CurveParams*);
void read_motor(MotorDynamics*);
void gain_scheduling(ControlVals *);
void execute_motor(MotorDynamics *);
void filter_motor(MotorDynamics *);
void read_params(CurveParams *);
void update_params(CurveParams *, CurveParams *);


void setup()
{
  pinMode(PIN_DIR_A, OUTPUT);
  pinMode(PIN_DIR_B, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  // pinMode(PIN_INVERT, INPUT);
  pinMode(PIN_LIMIT_SWITCH, INPUT_PULLUP);
  // pinMode(PIN_OPEN, INPUT_PULLUP);
  pinMode(PIN_REGULADOR_RANGO, INPUT_ANALOG);
  pinMode(PIN_REGULADOR_TIEMPO, INPUT_ANALOG);
  encoder.begin();
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A),  encoderISR,       CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B),  encoderISR,       CHANGE);
  lcd.begin(16, 2, LCD_5x8DOTS);
  lcd.setCursor(0,0);
  analogWriteFrequency(1000);
  digitalWrite(PIN_DIR_A, LOW);
  digitalWrite(PIN_DIR_B, HIGH);
  Serial.begin(115200);
  myservo.attach(PIN_PWM, 1000, 2000);
  // Serial.println("Booting Up...");
  // Serial.println(plus_c[0],5);
  // Serial.println(plus_c[1],5);
  // Serial.println(plus_c[2],5);
  // Serial.println(plus_c[3],5);
  // Serial.println(plus_c[4],5);
  // Serial.println(plus_c[5],5);
  // Serial.println(plus_c[6],5);
  // delay(10000);
} 

void loop() 
{
  BlinkLED();
  if(!cal_flag) calibrate();

  if(millis()>next_params_update)
  {
    read_params(&new_vals);
    update_params(&curve_vals, &new_vals);
    next_params_update = millis() + PARAMS_UPDATE_DELAY;
  }

  if(millis()>next_motor_update)
  {
    read_motor(&motor_vals);
    gain_scheduling(&control_vals);
    generate_curve(&motor_vals, &curve_vals);
    mimo_control(&motor_vals, &control_vals);
    filter_motor(&motor_vals);
    execute_motor(&motor_vals);
    next_motor_update = millis() + MOTOR_UPDATE_DELAY;
  }

  if (millis()>next_screen_update) 
  {
    lcd_vals.encoder_pos = encoder.getPosition() - zero_position;
    lcd_update(&lcd_vals);    
    next_screen_update = millis() + 500;
  }
}

void update_params(CurveParams *c, CurveParams *n)
{
  static bool init;
  if(!init)
  {
    // c->t[0] = ((-(sqrt(-1*(2*c->a_t*(c->accel[0]-c->accel[2]) + (c->accel[0]*c->accel[2]*((double)c->t_d/1000)*((double)c->t_d/1000)))*(c->accel[0]-c->accel[1])*(c->accel[1]-c->accel[2])))+((c->accel[0]- c->accel[1])*c->accel[2]*((double)c->t_d/1000))) / ((c->accel[0]-c->accel[2])*(c->accel[0]-c->accel[1])))*1000;
    // c->t[0] = ((-sqrt(-(2*c->a_t*(c->accel[0]-c->accel[2])+c->accel[0]*c->accel[2]*((double)(c->t_d)/1000)* (double)(c->t_d)/1000))*(c->accel[0]-c->accel[1])*(c->accel[1]-c->accel[2]))+(c->accel[0]-c->accel[1])*c->accel[2]* ((double)(c->t_d)/1000))/((c->accel[0]-c->accel[2])*(c->accel[0]-c->accel[1]))*1000;
    //c->t[0] = (((-sqrt (- (2*c->a_t* (c->accel[0]-c->accel[2] )+c->accel[0]*c->accel[2]*  (double)c->t_d /1000  *  (double)c->t_d /1000    )* (c->accel[0]-c->accel[1] )* (c->accel[1]-c->accel[2] ))+ (c->accel[0]-c->accel[1] )*c->accel[2]*  (double)c->t_d /1000 ))/(((c->accel[0]-c->accel[2] )* (c->accel[0]-c->accel[1] )) ))*1000;
    c->t[0] = 515;
    c->v[0] = (c->accel[0] * ((double)c->t[0]/1000))*DEG_TO_RAD;
    c->t[1] = (((double)c->t[0]/1000) * ((c->accel[1]-c->accel[0])/(c->accel[1]-c->accel[2])) - (((double)c->t_d/1000)*((c->accel[2])/(c->accel[1]-c->accel[2]))))*1000;
    c->v[1] = (-c->accel[2]*(((double)c->t_d-(double)c->t[1])/1000))*DEG_TO_RAD;
    c->t[2] = c->t_d;
    c->v[2] = 0;
    c->v[3] = 0;
    c->t[3] = c->t[2] + 1000;
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
    for(int i=0; i<7; i++)
    {
      Serial.print(c->t[i]);
      Serial.print(" ");
    }
    Serial.println();
    for(int i=0; i<7; i++)
    {
      Serial.print(c->v[i], 4);
      Serial.print(" ");
    }
    Serial.println();
    for(int i=0; i<7; i++)
    {
      Serial.print(c->plus_c[i], 4);
      Serial.print(" ");
    }
    Serial.println();
    delay(1000);

    init = 1;
  }
  return;
}

void read_params(CurveParams *c)
{
  return;
}
void generate_curve(MotorDynamics *m, CurveParams *c)
{
  static uint32_t initial_millis = millis();
  if(millis()-initial_millis >= 6000) initial_millis = millis();
  uint32_t current_step = (millis()-initial_millis);
  m->target_vel = get_target_velocity(current_step, c);
  m->target_pos = get_target_position(current_step, c);
  // Serial.print(current_step);
  // Serial.print(" ");
  Serial.print(m->target_vel*10);
  Serial.print(" ");
  Serial.println(m->target_pos*10);
}

void read_motor(MotorDynamics *m)
{
  static double motor_angular_position, motor_v_unf, prev_velocity;
  m->current_pos = (double) calculate_position();   //Position
  m->current_ang_pos = clicks_to_rad(m->current_pos);
  motor_v_unf = calculate_angular_velocity(motor_angular_position);
  m->current_vel = (double) ((motor_v_unf + prev_velocity * FILTER) / (FILTER + 1));  //Velocity
  prev_velocity = m->current_vel;
}

void gain_scheduling(ControlVals *c)
{
  static uint32_t initial_millis = millis();
  uint32_t current_step = (millis()-initial_millis)%(RISE_TIME/*+DEAD_TIME+FALL_TIME*/+WAITING_TIME);

  if(current_step < SECOND_TIME)
  {
    c->kp = K1;
    c->kv = K2;
  }
  else if (current_step < THIRD_TIME)
  {
    c->kp = K3; 
    c->kv = K4;
  }
  else
  {
    c->kp = K1_W;
    c->kv = K2_W;
  }

}

void filter_motor(MotorDynamics *m)
{
  static double prev_pwm;
  m->output = (m->output + prev_pwm * FILTER_PWM) / (FILTER_PWM + 1);
  prev_pwm = m->output;
  
  if(m->output > 500)
  {
    m->output = 500;
  }
  else if (m->output < -500)
  {
    m->output = -500;
  }

}

void execute_motor(MotorDynamics *m)
{
  motor_write(m->output);
}

void mimo_control(MotorDynamics *m, ControlVals *c)
{
  static double error_position, prx1,prx2,prx3;
  static double error_velocity;
  static double motor_volts;

  //Calculate error
  error_position = m->target_pos - m->current_ang_pos;
  error_velocity = m->target_vel - m->current_vel;

  //Error protection
  if (error_position > 0.8)
  {
    error_position = 0.8;
  }
  else if (error_position < -0.8)
  {
    error_position = -0.8;
  }

  motor_volts = c->kp * error_position + c->kv * error_velocity;
  m->output = map(motor_volts, 0, 12, 0, 500);


  // Serial.print(millis());
  // Serial.print(" ");
  // prx1= 100*error_position;
  // prx2 = 100*m->current_ang_pos;
  // prx3 = 100*m->target_pos;
  // Serial.print(prx1);
  // Serial.print(" ");
  // Serial.print(prx2);
  // Serial.print(" ");
  // Serial.print(prx3);
  // Serial.print(" ");
  // Serial.print(motor_volts,5);
  // Serial.print(" ");
  // Serial.print(error_velocity*100);
  // Serial.print(" ");
  // Serial.println(m->current_vel*100);
  //Serial.print(" ");
  // Serial.print(motor_pwm_filter);
  // Serial.print(" ");
  // Serial.println(target_position,5);
  //  Serial.print(" ");
  //  Serial.println(motor_volts,5);

  //Serial.println(millis());
  // Serial.println(target_position, 5);
  // Serial.println(error_position,5);
  // Serial.println(motor_pwm_filter);
  // Serial.println(motor_angular_position,5);
  // Serial.println(motor_volts);
  // Serial.println(target_velocity,5);
  
  // Serial.println(motor_velocity,5);
  // Serial.println(current_step,5);

  // Serial.println(error_velocity,5);
}

// double get_target_position(uint32_t current_step, double vel)
// {
//   static double target_pos;
//   static uint32_t prev_step = 0;
//   static double prev_vel = 0;
//   static double prev_pos = 0;
//   target_pos = prev_pos + 0.5*(vel + prev_vel)*((double)(current_step-prev_step)/1000);
//   prev_pos = target_pos;
//   prev_step = current_step;
//   prev_vel = vel;
//   // else
//   // {
//   //   target_vel = 0;
//   //   return target_vel;
//   // }
//   return target_pos;
// }

double get_target_position(uint32_t current_step, CurveParams *c)
{
  double target_pos;
  if(current_step <= c->t[0])
  {
    target_pos = 0.5*(c->v[0]/((double)c->t[0]))*((double)(current_step*current_step/1000)) + c->plus_c[0];
    return target_pos;
  }
  else if (current_step <= c->t[1])
  {
    target_pos = 0.5*((c->v[1] + c->v[0])/((double)(c->t[1] - c->t[0])/1000))*((double)(current_step - c->t[0])/1000)*((double)(current_step - c->t[0])/1000) + c->plus_c[1];
    return target_pos;
  }
  else if (current_step <= c->t[2])
  {
    target_pos = 0.5*((c->v[2] + c->v[1])/((double)(c->t[2] - c->t[1])/1000))*((double)(current_step - c->t[1])/1000)*((double)(current_step - c->t[1])/1000) + c->plus_c[2];
    return target_pos;
  }
  else if (current_step <= c->t[3])
  {
    target_pos = 0.5*((c->v[3] + c->v[2])/((double)(c->t[3] - c->t[2])/1000))*((double)(current_step - c->t[2])/1000)*((double)(current_step - c->t[2])/1000) + c->plus_c[3];
    return target_pos;
  }
  else if (current_step <= c->t[4])
  {
    target_pos = 0.5*((c->v[4] + c->v[3])/((double)(c->t[4] - c->t[3])/1000))*((double)(current_step - c->t[3])/1000)*((double)(current_step - c->t[3])/1000) + c->plus_c[4];
    return target_pos;
  }
  else if (current_step <= c->t[5])
  {
    target_pos = 0.5*((c->v[5] + c->v[4])/((double)(c->t[5] - c->t[4])/1000))*((double)(current_step - c->t[4])/1000)*((double)(current_step - c->t[4])/1000) + c->plus_c[5];
    return target_pos;
  }
  else
  {
    target_pos = 0.5*((c->v[6] + c->v[5])/((double)(c->t[6] - c->t[5])/1000))*((double)(current_step - c->t[5])/1000)*((double)(current_step - c->t[5])/1000) + c->plus_c[6];
    return target_pos;
  }
  return target_pos;
}

double get_target_velocity(uint32_t current_step, CurveParams *c)
{
  static double target_vel;
  if(current_step <= c->t[0])
  {
    target_vel = (c->v[0]/((double)c->t[0]))*((double)current_step);
    return target_vel;
  }
  else if (current_step <= c->t[1])
  {
    target_vel = c->v[0] + ((c->v[1] - c->v[0])/((double)(c->t[1] - c->t[0])))*((double)current_step - c->t[0]);
    return target_vel;
  }
  else if (current_step <= c->t[2])
  {
    target_vel = c->v[1] + ((c->v[2] - c->v[1])/((double)(c->t[2] - c->t[1])))*((double)current_step - c->t[1]);
    return target_vel;
  }
  else if (current_step <= c->t[3])
  {
    target_vel = 0;
    return target_vel;
  }
  else if (current_step <= c->t[4])
  {
    target_vel = c->v[3] + ((c->v[4] - c->v[3])/(c->t[4] - c->t[3]))*((double)current_step - c->t[3]);
    return target_vel;
  }
  else if (current_step <= c->t[5])
  {
    target_vel = c->v[4] + ((c->v[5] - c->v[4])/(c->t[5] - c->t[4]))*((double)current_step  - c->t[4]);
    return target_vel;
  }
  else
  {
    target_vel = 0;
    return target_vel;
  }
  return target_vel;
}

void BlinkLED() 
{
  static uint32_t next_blink = 0;
  if (millis() > next_blink)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    next_blink = millis() + BLINK_DELAY;
  }
}

void encoderISR()
{
  encoder.readAB();
}

void pulse_interrupt()
{
  // signal = digitalRead(PIN_SERIAL);
  // if(signal)
  // {
  //   start_pulse = micros();
  // }
  // else if(signal = 0)
  // {
  //   end_pulse = micros();
  //   pulse_width = end_pulse-start_pulse;
  // }
}

int16_t calculate_position()
{
  int16_t pos = encoder.getPosition() - zero_position;
  return pos;
}

double calculate_angular_velocity(double ang_pos)
{
  static double prev_angular_position;
  static uint32_t  old_millis;
  double velocity = 1000*(ang_pos - prev_angular_position)/(millis()- old_millis);
  prev_angular_position = ang_pos;
  old_millis = millis();
  
  return velocity;
}

int16_t calculate_angular_acceleration(double ang_vel)
{
  static int16_t prev_velocity = 0;
  int16_t acceleration = (ang_vel - prev_velocity)/MOTOR_UPDATE_DELAY;
  prev_velocity = ang_vel;
  return acceleration;
}

bool backlash_protection(double vel)
{
  static bool prev_dir, dir_change_flag;
  static int16_t backlash_init, backlash_end;
  if (vel > 0) dir = 1;
  else dir = 0;
  Serial.print("DIR: ");
  Serial.println(dir);
  if (prev_dir != dir)
  {
    prev_dir = dir;
    Serial.println("Dir change");
    dir_change_flag = 1;
    backlash_init = encoder.getPosition();
    if(dir)
    {
      backlash_end = backlash_init + BACKLASH_CLICKS;
      motor_write(100);
      Serial.println("Writting 100 to motor");
      Serial.print("Backlash end: ");
      Serial.println(backlash_end);
      return false;
    }
    else 
    {
      backlash_end = backlash_init - BACKLASH_CLICKS;
      Serial.println("Writting -100 to motor");
      Serial.print("Backlash end: ");
      Serial.println(backlash_end);
      motor_write(-100);
      return false;
    }
  }

  if (dir_change_flag && ((dir && encoder.getPosition() > backlash_end) || !dir && encoder.getPosition() < backlash_end)) 
  {
    dir_change_flag = false;
    return true;
  }

  else if (dir_change_flag == 0)
  {
    Serial.println("No changes: ");
    // Serial.print("Dir: ");
    // Serial.println(dir);
    return true;
  }

  else
  {
    Serial.println((encoder.getPosition() - backlash_end));
    return false;
  }
}

bool blocked_motor_protection(double ang_pos, double pwm)
{
  static double prev_ang_pos;
  static bool prev_dir;

  if (pwm > 0 /*&& prev_dir*/)
  {
    prev_dir = 1;
    if(ang_pos > (prev_ang_pos + PROTECTION_CLICKS)) 
    {
      prev_ang_pos = ang_pos;
      return true;
    }
    else 
    {
      prev_ang_pos = ang_pos;
      Serial.println("Blocked motor on positive movement");
      pwm = 0;
      return false;
    }
  }
  if(pwm < 0 /*&& !prev_dir*/)
  {
    Serial.println("Newgative");
   prev_dir = 0;
    if(ang_pos < (prev_ang_pos - PROTECTION_CLICKS)) 
    {
      prev_ang_pos = ang_pos;
      return true;
    }
    else
    {
      prev_ang_pos = ang_pos;
      Serial.println("Blocked motor on negative movement");
      pwm = 0;
      return false;
    }
  }
  return true;
}

void motor_write(double pwm)
{
  
  myservo.writeMicroseconds(1500+pwm);
}

double clicks_to_rad(int16_t clicks)
{
  return((double)(((double)360*((double)(3.1416/180))/(double)ENCODER_CPR)*(double)clicks));
}

int16_t deg_to_clicks(double deg)
{
  return((int16_t)((deg/360)*ENCODER_CPR));
}

double deg_to_rad(double deg)
{
  return((double)((deg*3.1416/180)));
}

void lcd_update(LcdVals *lcd_vals)
{
  static int16_t prev_enc_pos;
  if (lcd_vals->encoder_pos != prev_enc_pos)
  {
    lcd.clear();
    lcd.setCursor(0,1);
    lcd.print("Enc:");
    lcd.print(encoder.getPosition());
  }
  else return;
}

void calibrate()
{
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Calibrating...");
  while(digitalRead(PIN_LIMIT_SWITCH))
  {
    motor_write(-80);
    delay(1);
  }
  motor_write(0);
  zero_position = encoder.getPosition() + 100;
  lcd.setCursor(0,1);
  lcd.print("Done!");
  delay(500);
  lcd.setCursor(0,0);
  lcd.print("Place AMBUBAG");
  lcd.setCursor(0,1);
  lcd.print("between arms.");
  delay(5000);
  lcd.clear();
  cal_flag = 1;
}
// void super_calibrate()
// {
//   uint16_t prev_encoder_position;
//   bool is_first_loop, init_on_limit;
//   encoder.setPosition(48000);
//   prev_encoder_position = encoder.getPosition();

//   if(is_first_loop)
//   {
//     init_on_limit = digitalRead(PIN_LIMIT_SWITCH);
//     is_first_loop = 0;
//     for(int i=0;i<=CAL_TICKS;i++)
//     {
//       digitalWrite(PIN_DIR_A, HIGH);
//       digitalWrite(PIN_DIR_B, LOW);
//     }
//       digitalWrite(PIN_DIR_A, LOW);
//       digitalWrite(PIN_DIR_B, LOW);

    


//   }


// }

// void tick_movement()
// {void lcd_update(LcdVals *lcd_vals)

// }

