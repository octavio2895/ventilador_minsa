#include <Arduino.h>
#include <RotaryEncoder.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <math.h>
#include <Adafruit_ADS1015.h>
#include <PressureSensor.h>

// #define USBD_USE_CDC
// #define SCREEN

// Pin definitions.
#define PIN_ENCODER_A         PA7
#define PIN_ENCODER_B         PA6
#define PIN_PWM               PA15
#define PIN_DIR               PB1
#define PIN_CURRENT_SENSE     PB0
#define PIN_LIMIT_SWITCH      PA8

#define FORWARD_LOGIC         0
#define BACKWARD_LOGIC        1

#define USE_FLUTTER_PRINTS

// Physical constraints.
#define ENCODER_CPR           8000
#define ACCEL_1               320
#define ACCEL_2               -20
#define ACCEL_3               -160
#define ACCEL_4               0.23
#define MAX_ACCEL             320
#define ZERO_OFFSET           1000
#define MIN_VEL               60
#define MAX_VOL               50
#define MIN_VOL               0
#define MAX_RR                40
#define MIN_RR                0
#define MAX_X                 4
#define MIN_X                 1
#define MAX_PWM               256
#define KP_MAX                300
#define KP_MIN                80
#define KV_MAX                50
#define KV_MIN                0
#define CURRENT_COLLISION_THRESHOLD                200
#define AMBU_OPEN_ANGLE       55

// Update rates.
#define MOTOR_UPDATE_DELAY    10
#define MOTOR_SPEED_DELAY     20
#define SCREEN_UPDATE_DELAY   100
#define BLINK_DELAY           500
#define TARGET_UPDATE_DELAY   10
#define PARAMS_UPDATE_DELAY   100
#define SERIAL_UPDATE_DELAY   50
#define PRES_CAL_DELAY        100
#define SENSOR_UDPATE_DELAY   5
#define PLOT_UPDATE_DELAY     15

// Parameter
#define BREATH_PAUSE          0
#define MAX_ADC_RESOLUTION    16

//Experiment
#define FILTER                0
#define FILTER_PWM            0

#define K1                    37.5//85
#define K2                    1.25//4
#define K3                    120
#define K4                    0//4
#define K5                    150//4
#define K6                    0//4

//Target deg/s
#define DEG_TO_RAD            PI/180
#define CLICKS_TO_RAD         (2*PI/ENCODER_CPR)
#define RAD_TO_CLICK          (ENCODER_CPR/2*PI)

//Control encoder
struct ControlVals 
{
  double kp[7] = {K1, K3, K5, K1, K1, K1, K1};
  double kv[7] = {K2, K4, K6, K2, K2, K2, K2};;
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
  double rr = 15;
  double x = 1;
  double tidal_vol = 42;
  double a_t = tidal_vol;
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

PressureSensor pres_0  = {.id = 0}, pres_1 = {.id = 1};

// Global vars.
bool cal_flag = false, h_cal_flag = true, enc_inverted = false, dir, is_rising=1, is_falling=0, pause = 0, plot_flag = 0, params_change_flag = 0, pres_cal_fail = 0, stop_flag = 0, reset_flag = 0, restart_step_flag;
uint16_t zero_position = 0;
uint32_t next_motor_update = 0, next_speed_update = 0, next_dir_change, next_screen_update, next_params_update, next_serial_update, next_pres_cal, next_sensor_update, next_plot_update;
double pos_to_vel, flow, volume, volume_in, volume_out, pip, peep;

// Global objects.
RotaryEncoder encoder(PIN_ENCODER_B, PIN_ENCODER_A, PB0);
LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);  // Set the LCD I2C address
Adafruit_ADS1115 ads;

// Prototypes
void BlinkLED();
void calibrate();
void motor_write(double);
int16_t calculate_position();
double calculate_angular_velocity(MotorDynamics *);
int16_t calculate_angular_acceleration(double);
void encoderISR();
void mimo_control(MotorDynamics*, ControlVals*, StepInfo*);
double get_target_position(StepInfo*, CurveParams*);
double get_target_velocity(StepInfo*, CurveParams*);
void lcd_update(LcdVals *lcd_vals);
void generate_curve(StepInfo * , MotorDynamics *, CurveParams *);
void read_motor(MotorDynamics *);
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
int8_t k_check(double, double, double);
double calculate_volume(StepInfo *, double);
void home();
void reset_vals();
void get_accels(CurveParams*, double, double, double);
void hard_calibrate();

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_LIMIT_SWITCH, INPUT_PULLUP);
  pinMode(PIN_PWM, OUTPUT_OPEN_DRAIN);
  pinMode(PIN_DIR, OUTPUT);
  encoder.begin();
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A),  encoderISR,       CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B),  encoderISR,       CHANGE);
  #ifdef SCREEN
  lcd.begin(16, 2, LCD_5x8DOTS);
  lcd.setCursor(0,0);
  #endif
  analogReadResolution(12);
  analogWriteFrequency(20000);
  analogWrite(PIN_PWM, 128);
  Serial.begin(115200);
  Serial.println("Booting up...");
  ads.setGain(GAIN_SIXTEEN);
  ads.begin();
  // calibrate_pressure_sensor(&pres_0); // TODO: Review this
  // calibrate_pressure_sensor(&pres_1);
  pres_0.openpressure = 253;
  pres_1.openpressure = 13;
}

void loop() 
{
  BlinkLED();
  if(!cal_flag) calibrate();
  if(!h_cal_flag) hard_calibrate();

  if (millis() > next_sensor_update)
  {
    read_pressure(&pres_0);
    read_pressure_2(&pres_1);
    flow = calculate_flow_oplate(&pres_0);
    volume = calculate_volume(&step, flow);
    next_sensor_update = millis() + SENSOR_UDPATE_DELAY;
  }

  if(millis()>next_params_update || params_change_flag)
  {
    update_params(&step, &curve_vals, &new_vals);
    next_params_update = millis() + PARAMS_UPDATE_DELAY;
  }

  if(millis()>next_plot_update && plot_flag)
  {
    plot_data(&step, &curve_vals, &motor_vals);
    next_plot_update = millis() + PLOT_UPDATE_DELAY;
  }

  if(millis()>next_motor_update && !stop_flag && cal_flag)
  {
    calc_step(&step, &curve_vals);
    read_motor(&motor_vals);
    generate_curve(&step, &motor_vals, &curve_vals);
    mimo_control(&motor_vals, &control_vals, &step);
    filter_motor(&motor_vals);
    execute_motor(&motor_vals);
    next_motor_update = millis() + MOTOR_UPDATE_DELAY;
  }

  if(stop_flag && !reset_flag)
  {
    static bool first_run = 1;
    if(first_run)
    {
      reset_vals();
      first_run = false;
    }
    home();
    if (reset_flag) 
    {
      first_run = true;
      reset_flag = 0;
      pause = true;
    }
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

void reset_vals()
{
  restart_step_flag = true;
  volume = 0;
}

void home()
{
  Serial.println(encoder.getPosition()<<1);
  if(encoder.getPosition()<<1 == zero_position) reset_flag = 1;
  else if(encoder.getPosition()<<1 > zero_position) motor_write(-50);
  else if (encoder.getPosition()<<1 < zero_position) motor_write(50);
}

double calculate_volume(StepInfo *s, double flow)
{
  static Stages prev_stage = INS_1;
  static double volume = 0;
  static double prev_flow = 0;
  static double max_angle = 0;
  static double init_angle = 0;
  static uint32_t prev_millis = millis();
  static int16_t open_pressure_adc[16];
  static int16_t exp_press[16];
  static uint16_t open_pressure_index = 0;
  static double top_pres[64];
  static uint16_t top_pres_index = 0;

  if(prev_stage != INS_1 && s->cur_stage == INS_1)
  {
    volume = 0; //Resets volume to avoid drifting.
    Serial.print(max_angle, 5);
    Serial.print(" ");
    Serial.print(volume_in, 5);
    Serial.print(" ");
    Serial.print(pip, 5);
    Serial.print(" ");
    Serial.println(volume_out, 5);
    #ifdef USE_FLUTTER_PRINTS
    Serial.print("DATATOGRAPH: ");
    Serial.print("glen");
    Serial.print(curve_vals.t_f);
    Serial.println(",");
    #endif
  }
  volume = volume + (((prev_flow+flow)/2)*(millis() - prev_millis)/1000);
  if(prev_stage <= INS_3 && s->cur_stage >= REST_1)
  {
    max_angle = abs(((encoder.getPosition()<<1) - init_angle)*RAD_TO_DEG*CLICKS_TO_RAD);
    pip = arr_top(top_pres, 64);
    volume_in = volume;
  }

  else if(prev_stage == REST_2 && s->cur_stage == INS_1) 
  {
    volume_out = volume_in - volume;
    pres_0.openpressure = arr_average(open_pressure_adc, 16);
    peep = arr_average(exp_press, 16);
    init_angle = (encoder.getPosition()<<1);
  }

  if(s->cur_stage == INS_3)
  {
    if(top_pres_index > 63) top_pres_index = 0;
    top_pres[top_pres_index++] = pres_1.pressure;
  }

  if(s->cur_stage == REST_2)
  {
    if(open_pressure_index > 15) open_pressure_index = 0;
    exp_press[open_pressure_index] = (int16_t)pres_1.pressure;
    open_pressure_adc[open_pressure_index++] = (int16_t)pres_0.pressure_adc;
  }
  prev_millis = millis();
  prev_stage = s->cur_stage;
  prev_flow = flow;
  memset(top_pres, 0, sizeof(top_pres));
  return volume;
}

void parse_params(char buf[], uint16_t size, CurveParams *c, CurveParams *n)
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
    stop_flag = 1;
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
    h_cal_flag = 1;
    cal_flag = 0;
  }
  else if(!strcmp(cmd, "HCAL"))
  {
    cal_flag = 1;
    h_cal_flag = 0;
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
      get_accels(n, value_1, value_2, value_3);
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
  else if(!strcmp(cmd, "Kval"))
  {
    if (value_1 == 0 || value_2 == 0 || value_3 == 0)
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
      control_vals.kp[(uint16_t)value_1] = value_2;
      control_vals.kv[(uint16_t)value_1] = value_3;
    }
    else if (k_check_var == 1) Serial.println("Index out of range! Ignoring...");
    else if (k_check_var == 2) Serial.println("No negative values allowed! Ignoring...");
    else if (k_check_var == 3) Serial.println("KP is too high! Ignoring...");
    else if (k_check_var == 4) Serial.println("KP is too low! Ignoring...");
    else if (k_check_var == 5) Serial.println("KV is too high! Ignoring...");
    else if (k_check_var == 6) Serial.println("KV is too low! Ignoring...");
    Serial.println(control_vals.kp[(uint16_t)value_1]);
    Serial.println(control_vals.kv[(uint16_t)value_1]);
  }
  else Serial.println("CMD not recognized");
}

void get_accels(CurveParams *c, double rr, double x, double tidal_vol)
{
  double min_accel = rr*rr*tidal_vol*(x+1)*(x+1)/600; // TODO: redoing this calculation, this should be optimized.
  double max_accel = 17*rr*rr*tidal_vol*(x+1)*(x+1)/1800;
  c->accel[0] = max_accel > MAX_ACCEL ? (MAX_ACCEL + min_accel)/2 : (max_accel + min_accel)/2;
  c->accel[1] = -c->accel[0]/16;
  c->accel[2] = -c->accel[0]/2;
}

int8_t k_check(double stage, double kp, double kv)
{
  if(stage >= (sizeof(control_vals.kp)/sizeof(control_vals.kp[0]))) return 1;
  if(stage < 0 || kp < 0 || kv < 0) return 2;
  if(kp > KP_MAX) return 3;
  if(kp < KP_MIN) return 4;
  if(kv > KV_MAX) return 5;
  if(kv < KV_MIN) return 6;
  else return 0;
}


int8_t params_check(double rr, double x, double tidal_vol)
{
  if(tidal_vol > MAX_VOL) return 1;
  if(tidal_vol<MIN_VOL) return 2;
  if(rr>MAX_RR) return 3;
  if(rr<MIN_RR) return 4;
  if(x>MAX_X) return 5;
  if(x<MIN_X) return 6;
  double min_accel = rr*rr*tidal_vol*(x+1)*(x+1)/600;
  double max_accel = 17*rr*rr*tidal_vol*(x+1)*(x+1)/1800; // Actually, its just 5.666*min_accel. That probably depends on the constrain that m2 = -m1/16 and m3 = -m1/2.
  if(min_accel > MAX_ACCEL) return 7; // Cannot be run.
  else return 0; // Could be done by modifying acceleration.
  return -1; // Should never get here
}

void print_curve_data(CurveParams *c)
{
  Serial.print("RR: ");
  Serial.print(c->rr);
  Serial.print(" X: ");
  Serial.print(c->x);
  Serial.print(" Tidal Vol: ");
  Serial.println(c->tidal_vol);
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

void plot_data(StepInfo *s, CurveParams *c, MotorDynamics *m)
{
  #ifdef USE_FLUTTER_PRINTS
  static int16_t buffer_index = 0;
  static double graph_1[10];
  static double graph_2[10];
  static double graph_3[10];
  static uint32_t steps[10];
  graph_1[buffer_index] = volume;
  graph_2[buffer_index] = flow;
  graph_3[buffer_index] = pres_1.pressure;
  steps[buffer_index] = s->cur_step;
  if (buffer_index++ == 9)
  {
    for(int i = 0; i<(sizeof(steps)/sizeof(steps[0])); i++)
    {
      Serial.print("DATATOGRAPH: ");
      Serial.print("guno");
      Serial.print(graph_1[i], 5);
      Serial.print(",gdos");
      Serial.print(graph_2[i], 5);
      Serial.print(",gtres");
      Serial.print(graph_3[i], 5);
      Serial.print(",xxx");
      Serial.print(steps[i]);
      Serial.print(".00,");
      Serial.println();
    }
    Serial.print("DATATOGRAPH: ");
    Serial.print("rUUuno");
    Serial.print(volume_in, 4);
    Serial.print(",rdos");
    Serial.print(volume_out, 4);
    Serial.print(",rtres");
    Serial.print(pip, 2);
    Serial.print(",rcuatro");
    Serial.println(peep, 2);
    buffer_index = 0;
  }
  #else
  // Serial.print(pres_0.pressure_adc);
  // Serial.print(" ");
  // Serial.print(pres_0.openpressure);
  // Serial.print(" ");
  Serial.print(s->cur_step);
  Serial.print(" ");
  Serial.print(flow*60, 5);
  Serial.print(" ");
  Serial.print(volume, 5);
  Serial.print(" ");
  Serial.print(pres_0.pressure, 5);
  Serial.print(" ");
  Serial.print(pres_0.openpressure);
  Serial.print(" ");
  Serial.print(pres_0.pressure_adc, 5);
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
  Serial.print(" ");
  Serial.print(analogRead(PIN_CURRENT_SENSE));
  Serial.print(" ");
  Serial.print(c->t_f);
  Serial.println();
  #endif
}

void calc_step(StepInfo *s, CurveParams *c)
{
  static bool init = 0, was_paused = 0;
  if (restart_step_flag)
  {
    init = 0;
    s->cur_stage = INS_1;
    s->cur_step = 0;
    s->cur_millis = 0;
    restart_step_flag = 0;
  }
  if (!init)
  {
    s->start_millis = millis();
    init = true;
    restart_step_flag = 0;
  }
  if(was_paused && !pause)
  {
    s->start_millis = millis() - s->cur_step;
    was_paused = false;
  }
  if (!pause)
  {
    if(millis()-s->start_millis >=  c->t_f) s->start_millis = millis();
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
    c->accel[0] = n->accel[0];
    c->accel[1] = n->accel[1];
    c->accel[2] = n->accel[2];
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
  m->current_pos = (double) ((encoder.getPosition()<<1) - zero_position);
  m->current_ang_pos = CLICKS_TO_RAD * m->current_pos;
  m->current_vel = calculate_angular_velocity(m);
}

void filter_motor(MotorDynamics *m)
{
  static double prev_pwm;
  m->output = (m->output + prev_pwm * FILTER_PWM) / (FILTER_PWM + 1); // TODO: Review this
  prev_pwm = m->output;
  
  if(m->output > MAX_PWM)
  {
    m->output = MAX_PWM;
  }
  else if (m->output < -MAX_PWM)
  {
    m->output = -MAX_PWM;
  }

}

void execute_motor(MotorDynamics *m)
{
  if (pause)
  {
    analogWrite(PIN_PWM, 0);
    return;
  }
  else motor_write(m->output);
}

void mimo_control(MotorDynamics *m, ControlVals *c, StepInfo *s)
{
  static double error_position;
  static double error_velocity;
  static double motor_volts;

  //Calculate error
  error_position = m->target_pos - m->current_ang_pos;
  error_velocity = m->target_vel - m->current_vel;

  //Error protection
  if (error_position > 0.8) // TODO: Review error protection
  {
    error_position = 0.8;
  }
  else if (error_position < -0.8)
  {
    error_position = -0.8;
  }

  motor_volts = c->kp[s->cur_stage] * error_position + c->kv[s->cur_stage] * error_velocity;
  m->output = fmap(motor_volts, 0, 12, 0, MAX_PWM);

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
  int16_t pos = encoder.getPosition()<<1 - zero_position;
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
  int16_t acceleration = (ang_vel - prev_velocity)/MOTOR_UPDATE_DELAY;
  prev_velocity = ang_vel;
  return acceleration;
}

void motor_write(double pwm)
{
  if(pwm > 0) digitalWrite(PIN_DIR, 0);
  else digitalWrite(PIN_DIR, 1);
  analogWrite(PIN_PWM, abs(pwm));
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
  static uint8_t state = 0;
  static uint16_t stopped_millis = 0;

  switch (state)
  {
  case 0:
    pause = 1;
    Serial.print("Calibrating...");
    state++;
    break;
  case 1:
    if(digitalRead(PIN_LIMIT_SWITCH))
    {
      motor_write(-MIN_VEL);
    }
    else
    {
      state++;
      motor_write(0);
    }
    break;
  case 2:
    Serial.println(" Done!");
    Serial.println(encoder.getPosition()<<1);
    encoder.setPosition(0);
    zero_position = (encoder.getPosition()<<1) + ZERO_OFFSET;
    Serial.println(zero_position);
    stopped_millis = millis() + 5000;
    state++;
    break;
  case 3:
    if(millis() > stopped_millis) 
    {
      Serial.print("Going to zero...");
      state++;
    }
    break;
  case 4:
    if(((encoder.getPosition()<<1) - zero_position) < 0)
    {
      Serial.println((encoder.getPosition()<<1) - zero_position);
      motor_write(MIN_VEL);
    }
    else
    {
      state++;
      motor_write(0);
    }
    break;
  case 5:
    Serial.println(" Done!");
    cal_flag = 1;
    state = 0;
    break;
  default:
    break;
  }
}

void hard_calibrate()
{
  static uint8_t state = 0;
  static uint16_t stopped_millis = 0;

  switch (state)
  {
  case 0:
    pause = 1;
    Serial.println("Performing hard calibration...");
    state++;
    break;
  case 1:
    if(digitalRead(PIN_LIMIT_SWITCH))
    {
      motor_write(-MIN_VEL);
    }
    else
    {
      state++;
      motor_write(0);
    }
    break;
  case 2:
    Serial.println(" Done!");
    Serial.println(encoder.getPosition()<<1);
    encoder.setPosition(0);
    // zero_position = (encoder.getPosition()<<1) + ZERO_OFFSET;
    // Serial.println(zero_position);
    stopped_millis = millis() + 5000;
    state++;
    break;
  case 3:
    if(millis() > stopped_millis) 
    {
      Serial.print("Going to upper limit...");
      state++;
    }
    break;
  case 4:
  {
    uint32_t current_adc = analogRead(PIN_CURRENT_SENSE);
    if(current_adc < CURRENT_COLLISION_THRESHOLD)
    {
      Serial.println((encoder.getPosition()<<1) - zero_position);
      Serial.println(current_adc);
      motor_write(MIN_VEL);
    }
    else
    {
      motor_write(0);
      state++;
      Serial.print("Upper limit: ");
      Serial.println(encoder.getPosition());
      zero_position = encoder.getPosition() - AMBU_OPEN_ANGLE*DEG_TO_RAD*RAD_TO_CLICK;
      Serial.println("Going to zero...");
    }
    break;
  }
  case 5:
    if(((encoder.getPosition()<<1) - zero_position) > 0)
    {
      Serial.println((encoder.getPosition()<<1) - zero_position);
      motor_write(-MIN_VEL);
    }
    else
    {
      state++;
      motor_write(0);
    }
    break;
  case 6:
    Serial.println(" Done!");
    h_cal_flag = 1;
    state = 0;
    break;
  default:
    break;
  }
}