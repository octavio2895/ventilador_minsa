#include <Arduino.h>
#include <RotaryEncoder.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <math.h>
#include <Adafruit_ADS1015.h>


// Pin definitions.
#define PIN_ENCODER_A         PA7
#define PIN_ENCODER_B         PA6
#define PIN_PWM               PA15
#define PIN_LIMIT_SWITCH      PA8

// Physical constraints.
#define ROTARYMIN             0
#define ROTARYMAX             200
#define ROTARYINITIAL         0
#define ENCODER_CPR           8000
#define MOTOR_DZ              0 //165
#define BACKLASH_CLICKS       5
#define PROTECTION_CLICKS     5
#define ACCEL_1               16
#define ACCEL_2               -1
#define ACCEL_3               -8
#define ACCEL_4               0.23
#define ZERO_OFFSET           0.009*ENCODER_CPR
#define MIN_VEL               80
#define VENTURI_SMALL_DIAM    9e-3f
#define VENTURI_BIG_DIAM      22.5e-3f
#define AIR_DENSITY           1.2f
#define VENTURI_SMALL_AREA    (VENTURI_SMALL_DIAM*VENTURI_SMALL_DIAM/4)*PI
#define VENTURI_BIG_AREA      (VENTURI_BIG_DIAM*VENTURI_BIG_DIAM/4)*PI
#define MULTIPLIER            0.0078125f
#define MAX_VOL               10
#define MIN_VOL               0
#define MAX_RR                15
#define MIN_RR                0
#define MAX_X                 4
#define MIN_X                 1

// Update rates.
#define MOTOR_UPDATE_DELAY    10
#define MOTOR_SPEED_DELAY     20
#define SCREEN_UPDATE_DELAY   100
#define BLINK_DELAY           500
#define TARGET_UPDATE_DELAY   10
#define PARAMS_UPDATE_DELAY   100
#define SERIAL_UPDATE_DELAY   50
#define PRES_CAL_DELAY        100
#define SENSOR_UDPATE_DELAY   30

// Parameter
#define CAL_PWM               10
#define CAL_DRIVE_BACK_ANG    5
#define CAL_DRIVE_ANG         0.5
#define CAL_DZ                0.5
#define BREATH_PAUSE          0

//Super Calibrate
#define CAL_TICKS             5

//Experiment
#define RISE_TIME             3000
#define DEAD_TIME             0//1000
#define FALL_TIME             2000
#define WAITING_TIME          2000
#define ACC_TIME              800
#define DEGREES               360
#define FILTER                0
#define FILTER_PWM            0

#define K1                    150//85
#define K2                    10//4
#define K3                    150
#define K4                    0//4
#define K1_W                  35
#define K2_W                  0//1
//Targets times
#define FIRST_TIME            600
#define SECOND_TIME           2000
#define THIRD_TIME            3000

//Target degrees
#define F_DEG                 5
#define S_DEG                 11
#define T_DEG                 15

//Target deg/s
#define DEG_P_SEG             8
#define DEG_TO_RAD            PI/180
#define CLICKS_TO_RAD         (2*PI/ENCODER_CPR)

// #define SCREEN

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
  volatile uint32_t click_time = 1; // Avoid div 0
  volatile bool click_dir;
}motor_vals;

struct CurveParams
{
  double const_vel_time_rise = RISE_TIME - 2*ACC_TIME;
  double const_vel_time_fall = FALL_TIME - 2*ACC_TIME;
  double theta_dot_max_rise = DEGREES/(const_vel_time_rise+ACC_TIME);
  double theta_dot_max_fall = 36/10;
  double rr = 10;
  double x = 1;
  double tidal_vol = 6;
  double a_t = tidal_vol;
  uint32_t t_f = 60000/rr;
  uint32_t t_d = t_f/(x+1);

  double accel[3] = 
  {
    ACCEL_1,
    ACCEL_2,
    ACCEL_3
  };

  double v[7];
  uint32_t t[7];
  double plus_c[7];

}curve_vals, new_vals;


enum Stages
{
  INS_1 = 0,
  INS_2 = 1,
  INS_3 = 2,
  REST_1 = 3,
  EXP_1 = 4,
  EXP_2 = 5,
  REST_2 = 6
};


struct StepInfo
{
  uint32_t start_millis;
  uint32_t cur_millis;
  Stages cur_stage;
  uint32_t cur_step;
}step;

struct PlotDat
{
  uint32_t cur_step;
  double err_pos;
  double err_vel;
  double tar_pos;
  double tar_vel;
  double cur_pos;
  double cur_vel;
}plot;

struct PressureSensor
{
  uint8_t id;
  int16_t openpressure;
  int16_t pressure_adc;
  double pressure;
}pres_0  = {.id = 0}, pres_1 = {.id = 1};

// Global vars.

bool cal_flag = false, enc_inverted = false, dir, is_rising=1, is_falling=0, pause = 0, plot_flag = 0, params_change_flag = 0, pres_cal_fail = 0;
uint16_t zero_position = 0;
uint32_t next_motor_update = 0, next_speed_update = 0, next_dir_change, next_screen_update, next_params_update, next_serial_update, next_pres_cal, next_sensor_update;
double pos_to_vel, flow, volume;

// Global objects.
RotaryEncoder encoder(PIN_ENCODER_A, PIN_ENCODER_B, PB0);
LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);  // Set the LCD I2C address
Servo myservo;
Adafruit_ADS1115 ads;

// Prototypes
void change_control_values(ControlVals *vals);
void BlinkLED();
void calibrate();
void motor_set_dir(double);
void motor_write(double);
int16_t calculate_position();
double calculate_angular_velocity(MotorDynamics *);
int16_t calculate_angular_acceleration(double);
void encoderISR();
float arr_average(float *arr, uint16_t size);
void mimo_control(MotorDynamics*, ControlVals*);
double get_target_position(StepInfo*, CurveParams*);
double get_target_velocity(StepInfo*, CurveParams*);
void pulse_interrupt();
void lcd_update(LcdVals *lcd_vals);
bool backlash_protection(double);
bool blocked_motor_protection(double, double);
void generate_curve(StepInfo * , MotorDynamics *, CurveParams *);
void read_motor(MotorDynamics *);
void gain_scheduling(ControlVals *);
void execute_motor(MotorDynamics *);
void filter_motor(MotorDynamics *);
void read_params(CurveParams *);
void update_params(StepInfo *, CurveParams *, CurveParams *);
void plot_data(StepInfo *, CurveParams *, MotorDynamics *);
void calc_step(StepInfo *, CurveParams *);
double fmap(double in, double in_min, double in_max, double out_min, double out_max);
void parse_params(char buf[], uint16_t size, CurveParams *c, CurveParams *n);
void print_curve_data(CurveParams*);
int8_t params_check(double, double, double);
int16_t calibrate_pressure_sensor(PressureSensor *);
void read_pressure(PressureSensor *);
double calculate_flow(PressureSensor *);
double calculate_volume(StepInfo *, double);


void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_LIMIT_SWITCH, INPUT_PULLUP);
  encoder.begin();
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A),  encoderISR,       CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B),  encoderISR,       CHANGE);
  #ifdef SCREEN
  lcd.begin(16, 2, LCD_5x8DOTS);
  lcd.setCursor(0,0);
  #endif
  analogWriteFrequency(1000);
  Serial.begin(115200);
  myservo.attach(PIN_PWM, 1000, 2000);
  Serial.println("Booting up...");
  ads.setGain(GAIN_SIXTEEN);
  ads.begin();
  // if(calibrate_pressure_sensor(&pres_0) == 0  || calibrate_pressure_sensor(&pres_1) == 0) pres_cal_fail = 1;
  // Serial.println(ads.readADC_Differential_0_1());
  // Serial.println(ads.readADC_Differential_2_3());
}

void loop() 
{
  BlinkLED();
  // if(!cal_flag) calibrate();
  // if(pres_cal_fail && millis() > next_pres_cal)
  // {
  //   Serial.println("Pressure sensor cannot calibrate!");
  //   if(calibrate_pressure_sensor(&pres_0) == 0  || calibrate_pressure_sensor(&pres_1) == 0) pres_cal_fail = 1;
  //   else pres_cal_fail = 0;
  //   next_pres_cal = millis() + PRES_CAL_DELAY;
  // }

  if (millis() > next_sensor_update)
  {
    read_pressure(&pres_0);
    read_pressure(&pres_1);
    flow = calculate_flow(&pres_1);
    volume = calculate_volume(&step, flow);
    next_sensor_update = millis() + SENSOR_UDPATE_DELAY;
  }

  if(millis()>next_params_update || params_change_flag)
  {
    // read_params(&new_vals);
    update_params(&step, &curve_vals, &new_vals);
    next_params_update = millis() + PARAMS_UPDATE_DELAY;
  }

  if(millis()>next_motor_update)
  {
    calc_step(&step, &curve_vals);
    read_motor(&motor_vals);
    gain_scheduling(&control_vals);
    generate_curve(&step, &motor_vals, &curve_vals);
    mimo_control(&motor_vals, &control_vals);
    filter_motor(&motor_vals);
    execute_motor(&motor_vals);
    plot_data(&step, &curve_vals, &motor_vals);
    next_motor_update = millis() + MOTOR_UPDATE_DELAY;
  }
  #ifdef SCREEN
  if (millis()>next_screen_update) 
  {
    lcd_vals.encoder_pos = encoder.getPosition() - zero_position;
    lcd_update(&lcd_vals);    
    next_screen_update = millis() + 500;
  }
  #endif

  if (millis()>next_serial_update)
  { 
    if(Serial.available())
    { 
      char serial_buf [100];
      memset(serial_buf, 0x00, sizeof(serial_buf)/sizeof(char));
      int i = 0;
      while (Serial.available())
      {

        serial_buf[i++] = Serial.read();
      }
      parse_params(serial_buf, (sizeof(serial_buf)/sizeof(char)), &curve_vals, &new_vals);
      next_serial_update = millis() + SERIAL_UPDATE_DELAY;
    }
    else next_serial_update = millis() + SERIAL_UPDATE_DELAY;
  }
}

double calculate_volume(StepInfo *s, double flow)
{
  static Stages prev_stage = INS_1;
  static double volume = 0;
  static double prev_flow = 0;
  static uint32_t prev_millis = millis();

  if(prev_stage != INS_1 && s->cur_stage == INS_1) volume = 0; //Resets volume to avoid drifting.
  volume = volume + (((prev_flow+flow)/2)*(millis() - prev_millis)/1000);
  prev_millis = millis();
  prev_stage = s->cur_stage;
  return volume;
}

double calculate_flow(PressureSensor *p)
{
  return (sqrt((2 * abs(p->pressure)) / (AIR_DENSITY * ((VENTURI_BIG_AREA / VENTURI_SMALL_AREA) * (VENTURI_BIG_AREA / VENTURI_SMALL_AREA) - 1))));
}

void read_pressure(PressureSensor *p)
{
  if(p->id == 1) p->pressure_adc = ((float) ads.readADC_Differential_2_3());
  else p->pressure_adc = ((float) ads.readADC_Differential_0_1());
  p->pressure = (p->pressure_adc - p->openpressure) * MULTIPLIER * 5;
  if (p->pressure_adc >= floor(p->openpressure) - 1 && p->pressure_adc <= floor(p->openpressure) + 1) 
  {
    p->pressure = 0;
  }
  p->pressure = 1000*abs(p->pressure);
}

void parse_params(char buf[], uint16_t size, CurveParams *c, CurveParams *n)
{
  char cmd[10];
  int32_t u_value_1 = 0;
  int32_t u_value_2 = 0;
  int32_t u_value_3 = 0;
  char temp_buf[100];
  memset(temp_buf, 0x00, sizeof(temp_buf)/sizeof(char));
  Serial.println(sscanf(buf, " %s %d %d %d ", cmd, &u_value_1, &u_value_2, &u_value_3));
  double value_1 = (double)u_value_1/10;
  double value_2 = (double)u_value_2/10;
  double value_3 = (double)u_value_3/10;

  if(!strcmp(cmd, "PING"))
  {
    Serial.println("PING");
  }
  else if(!strcmp(cmd, "PAUSE"))
  {
    Serial.println("Pausing the machine...");
    pause = 1;
  }
  else if(!strcmp(cmd, "RESUME"))
  {
    Serial.println("Resuming the machine...");
    pause = 0;
  }
  else if(!strcmp(cmd, "CAL"))
  {
    cal_flag = 0;
  }
  else if(!strcmp(cmd, "PLOT"))
  {
    plot_flag = !plot_flag;
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

    int param_check_var = params_check(value_1, value_2, value_3);
    if(param_check_var == 0)
    {
      n->rr = value_1;
      n->x = value_2;
      n->tidal_vol = value_3;
      params_change_flag = 1;
    }
    else if (param_check_var == 1) Serial.println("Tidal volume too high! Ignoring...");
    else if (param_check_var == 2) Serial.println("Tidal volume too low! Ignoring...");
    else if (param_check_var == 3) Serial.println("Respiration rate too high!, ignoring...");
    else if (param_check_var == 4) Serial.println("Respiration rate too low!, ignoring...");
    else if (param_check_var == 5) Serial.println("Respiration relation too high!, ignoring...");
    else if (param_check_var == 6) Serial.println("Respiration ralation too low!, ignoring...");
    else Serial.println("Impossible combination, ignoring...");
  }
  else Serial.println("CMD not recognized");
}

int8_t params_check(double rr, double x, double tidal_vol)
{
  uint32_t t_f = 60000/rr;
  uint32_t t_d = t_f/(x+1);
  if(tidal_vol > MAX_VOL) return 1;
  if(tidal_vol<MIN_VOL) return 2;
  if(rr>MAX_RR) return 3;
  if(rr<MIN_RR) return 4;
  if(x>MAX_X) return 5;
  if(x<MIN_X) return 6;
  if(tidal_vol > (-0.5*ACCEL_1*ACCEL_3*((double)t_d/1000)*((double)t_d/1000)/(ACCEL_1-ACCEL_3))) return 7;
  if(t_d > 1000*sqrt((-2*tidal_vol*(ACCEL_1 - ACCEL_2))/(ACCEL_1*ACCEL_2))) return 8;
  return false;
}

void print_curve_data(CurveParams *c)
{
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
}

void plot_data(StepInfo *s, CurveParams *c, MotorDynamics *m)
{
  if(!plot_flag) return;
  Serial.print(s->cur_step);
  Serial.print(" ");
  Serial.print(volume, 5);
  Serial.print(" ");
  Serial.print(flow, 5);
  Serial.print(" ");
  Serial.print(pres_1.pressure, 5);
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
  Serial.println();
}

void calc_step(StepInfo *s, CurveParams *c)
{
  static bool init = 0, was_paused = 0;
  if (!init)
  {
    s->start_millis = millis();
    init = true;
  }
  if(was_paused && !pause)
  {
    s->start_millis = millis() - s->cur_step;
    was_paused = false;
  }
  if (!pause)
  {
    if(millis()-s->start_millis >= 6000) s->start_millis = millis();
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

void update_params(StepInfo *s, CurveParams *c, CurveParams *n)
{
  static bool init;
  if (params_change_flag && s->cur_step < 20)
  {
    c->tidal_vol = n->tidal_vol;
    c->rr = n->rr;
    c->x = n->x;
    c->a_t = c->tidal_vol;
    c->t_f = 60000/c->rr;
    c->t_d = c->t_f/(c->x+1);
    init = 0;
    params_change_flag = 0;
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
    print_curve_data(c);
    init = 1;
  }
  return;
}

void read_params(CurveParams *c)
{
  return;
}
void generate_curve(StepInfo *s, MotorDynamics *m, CurveParams *c)
{
  if(!pause)
  {
    m->target_vel = get_target_velocity(s, c);
    m->target_pos = get_target_position(s, c);
  }
}

void read_motor(MotorDynamics *m)
{
  m->current_pos = (double) calculate_position();
  m->current_ang_pos = CLICKS_TO_RAD * m->current_pos;
  m->current_vel = calculate_angular_velocity(m);
}

void gain_scheduling(ControlVals *c)
{
  c->kp = K1;
  c->kv = K2;
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
  if (pause)
  {
    motor_write(0);
    return;
  }
  else motor_write(m->output);
}

void mimo_control(MotorDynamics *m, ControlVals *c)
{
  static double error_position;
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
  m->output = fmap(motor_volts, 0, 12, 0, 500);

}

double fmap(double in, double in_min, double in_max, double out_min, double out_max)
{
  return((in/(in_max-in_min))*(out_max-out_min)+out_min);
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

int16_t calculate_position()
{
  int16_t pos = encoder.getPosition() - zero_position;
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

double calculate_angular_velocity_fod3(MotorDynamics *m)
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
  // lcd.clear();
  // lcd.setCursor(0,0);
  // lcd.print("Calibrating...");
  Serial.print("Calibrating... ");
  while(digitalRead(PIN_LIMIT_SWITCH))
  {
    motor_write(-MIN_VEL);
    delay(1);
  }
  motor_write(0);
  Serial.println("Done!");
  zero_position = encoder.getPosition() + 180;
  // lcd.setCursor(0,1);
  // lcd.print("Done!");
  delay(500);
  // lcd.setCursor(0,0);
  // lcd.print("Place AMBUBAG");
  // lcd.setCursor(0,1);
  // lcd.print("between arms.");
  Serial.println(encoder.getPosition());
  Serial.println(calculate_position());
  delay(5000);
  while(calculate_position() < 0)
  {
    Serial.println(calculate_position());
    motor_write(MIN_VEL);
    delay(1);
  }
  motor_write(0);
  delay(4000);
  // lcd.clear();
  cal_flag = 1;
}

int16_t calibrate_pressure_sensor(PressureSensor *p) 
{
  int count = 0;
  uint32_t timer = 0;
  double open_pressure;
  double average = 0;

  while (true)
  {
    if (timer < millis()) 
    {
      if (p->id == 0) open_pressure = (double)ads.readADC_Differential_0_1();
      if (p->id == 1) open_pressure = (double)ads.readADC_Differential_2_3();
      average = (average * (double)count + open_pressure) / ((double)count + 1);
      count++;
      timer = millis() + 10;;
    }
    // if (count > 50) 
    // {
    //   if ((open_pressure > (average * 1.1f)) || (open_pressure < (average * 0.9f))) 
    //   {
    //     p->openpressure = 0;
    //     return 0;
    //   }
    // }
    if (count >= 200) 
    {
      p->openpressure = average;
      Serial.println(p->openpressure);
      break;
    }
  }  

  return ((int16_t)average);
}