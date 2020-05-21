#include <Arduino.h>
#include <RotaryEncoder.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>


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
#define ACCEL_1 0.23
#define ACCEL_2 -0.02
#define ACCEL_3 -0.14
#define ACCEL_4 0.23

// Update rates.
#define MOTOR_UPDATE_DELAY    10
#define MOTOR_SPEED_DELAY     20
#define SCREEN_UPDATE_DELAY   100
#define BLINK_DELAY           500
#define TARGET_UPDATE_DELAY   10

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
  double accel_1 = 0.23;
  double accel_2 = -1.746e-5;
  double accel_3 = -.02;
  double accel_4 = 0.23;
  double v_1 = 0.14428637;
  double v_2 = 0.13488079;
  double v_3 = 0;
  double v_4 = 0;
  double v_5 = -0.14428637;
  double v_6 = 0;
  double v_7 = 0;
  uint32_t t_1 = 515;
  uint32_t t_2 = 1034;
  uint32_t t_3 = 2000;
  uint32_t t_4 = 3000;
  uint32_t t_5 = 4117;
  uint32_t t_6 = 5234;
  uint32_t t_7 = 6000;
}curve_vals;

// Global vars.
double plus_c[]
{
  0,
  0.5*(curve_vals.v_1 * ((double)(curve_vals.t_1)/1000)),
  plus_c[1] + (0.5*(double)(curve_vals.v_2 + curve_vals.v_1)*((double)(curve_vals.t_2 - curve_vals.t_1)/1000)),
  plus_c[2] + (0.5*(double)(curve_vals.v_3 + curve_vals.v_2)*((double)(curve_vals.t_3 - curve_vals.t_2)/1000)),
  plus_c[3] + (0.5*(double)(curve_vals.v_4 + curve_vals.v_3)*((double)(curve_vals.t_4 - curve_vals.t_3)/1000)),
  plus_c[4] + (0.5*(double)(curve_vals.v_5 + curve_vals.v_4)*((double)(curve_vals.t_5 - curve_vals.t_4)/1000)),
  plus_c[5] + (0.5*(double)(curve_vals.v_6 + curve_vals.v_5)*((double)(curve_vals.t_6 - curve_vals.t_5)/1000))
};
bool cal_flag = false, enc_inverted = false, dir, is_rising=1, is_falling=0;
uint16_t zero_position = 0;
uint32_t next_motor_update = 0, next_speed_update = 0, next_dir_change, next_screen_update, next_kp_update;
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
  Serial.println(plus_c[0],5);
  Serial.println(plus_c[1],5);
  Serial.println(plus_c[2],5);
  Serial.println(plus_c[3],5);
  Serial.println(plus_c[4],5);
  Serial.println(plus_c[5],5);
  Serial.println(plus_c[6],5);
  delay(10000);
} 

void loop() 
{
  BlinkLED();
  // if(!cal_flag) calibrate();

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
  if(current_step <= c->t_1)
  {
    target_pos = 0.5*(c->v_1/((double)c->t_1))*((double)(current_step*current_step/1000)) + plus_c[0];
    return target_pos;
  }
  else if (current_step <= c->t_2)
  {
    target_pos = 0.5*((c->v_2 + c->v_1)/((double)(c->t_2 - c->t_1)/1000))*((double)(current_step - c->t_1)/1000)*((double)(current_step - c->t_1)/1000) + plus_c[1];
    return target_pos;
  }
  else if (current_step <= c->t_3)
  {
    target_pos = 0.5*((c->v_3 + c->v_2)/((double)(c->t_3 - c->t_2)/1000))*((double)(current_step - c->t_2)/1000)*((double)(current_step - c->t_2)/1000) + plus_c[2];
    return target_pos;
  }
  else if (current_step <= c->t_4)
  {
    target_pos = 0.5*((c->v_4 + c->v_3)/((double)(c->t_4 - c->t_3)/1000))*((double)(current_step - c->t_3)/1000)*((double)(current_step - c->t_3)/1000) + plus_c[3];
    return target_pos;
  }
  else if (current_step <= c->t_5)
  {
    target_pos = 0.5*((c->v_5 + c->v_4)/((double)(c->t_5 - c->t_4)/1000))*((double)(current_step - c->t_4)/1000)*((double)(current_step - c->t_4)/1000) + plus_c[4];
    return target_pos;
  }
  else if (current_step <= c->t_6)
  {
    target_pos = 0.5*((c->v_6 + c->v_5)/((double)(c->t_6 - c->t_5)/1000))*((double)(current_step - c->t_5)/1000)*((double)(current_step - c->t_5)/1000) + plus_c[5];
    return target_pos;
  }
  else
  {
    target_pos = 0.5*((c->v_7 + c->v_6)/((double)(c->t_7 - c->t_6)/1000))*((double)(current_step - c->t_6)/1000)*((double)(current_step - c->t_6)/1000) + plus_c[6];
    return target_pos;
  }
  return target_pos;
}

double get_target_velocity(uint32_t current_step, CurveParams *c)
{
  static double target_vel;
  if(current_step <= c->t_1)
  {
    target_vel = (c->v_1/((double)c->t_1))*((double)current_step);
    return target_vel;
  }
  else if (current_step <= c->t_2)
  {
    target_vel = c->v_1 + ((c->v_2 - c->v_1)/((double)(c->t_2 - c->t_1)))*((double)current_step - c->t_1);
    return target_vel;
  }
  else if (current_step <= c->t_3)
  {
    target_vel = c->v_2 + ((c->v_3 - c->v_2)/((double)(c->t_3 - c->t_2)))*((double)current_step - c->t_2);
    return target_vel;
  }
  else if (current_step <= c->t_4)
  {
    target_vel = 0;
    return target_vel;
  }
  else if (current_step <= c->t_5)
  {
    target_vel = c->v_4 + ((c->v_5 - c->v_4)/(c->t_5 - c->t_4))*((double)current_step - c->t_4);
    return target_vel;
  }
  else if (current_step <= c->t_6)
  {
    target_vel = c->v_5 + ((c->v_6 - c->v_5)/(c->t_6 - c->t_5))*((double)current_step  - c->t_5);
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

