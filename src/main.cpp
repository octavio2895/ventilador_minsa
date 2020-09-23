#include <Arduino.h>
#include <RotaryEncoder.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <Adafruit_ADS1015.h>
#include <ODriveArduino.h>

#include <FlowState.h>
#include <Curves.h>
#include <StepData.h>
#include <SerialInterface.h>
#include <MotorController.h>
#include <SysState.h>
#include <Encoder.h>
#include <SysStructs.h>

// #define USBD_USE_CDC
// #define SCREEN

// Pin definitions.
#define PIN_ENCODER_A         PA7
#define PIN_ENCODER_B         PA6
#define PIN_PWM               PA15
#define PIN_DIR               PB0
#define PIN_CURRENT_SENSE     PB1
#define PIN_LIMIT_SWITCH      PA8

// Physical constraints.
#define ENCODER_CPR           8000
#define ACCEL_1               320
#define ACCEL_2               -20
#define ACCEL_3               -160
#define MAX_ACCEL             700
#define ZERO_OFFSET           750
#define MIN_VEL               60
#define MAX_VOL               55
#define MIN_VOL               0
#define MAX_RR                40
#define MIN_RR                0
#define MAX_X                 4
#define MIN_X                 1
#define CURRENT_COLLISION_THRESHOLD                200
#define AMBU_OPEN_ANGLE       70

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
#define PLOT_UPDATE_DELAY     5
#define ERROR_UPDATE_DELAY    100

// Parameter
#define MAX_ADC_RESOLUTION    16
#define MAX_MOTOR_NUM         1
#define MAX_RESTART_RETRIES   3
#define MAX_ODRIVE_CAL_RETRIES 3
#define LIMIT_SWITCH_LOGIC    1
#define HOMING_SPEED          5000
#define HOME_DZ               1000



struct LcdVals
{
  int16_t encoder_pos;
}lcd_vals;

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

FlowData current_flow;
PressureSensor pres_0  = {.id = 0}, pres_1 = {.id = 1};
SysState sys_state;
ControlVals control_vals;
StepInfo step;
MotorDynamics motor_vals;// = {.dir_pin = PIN_DIR, .pwm_pin = PIN_PWM};
CurveParams curve_vals, new_vals;
HardwareSerial Serial2(PA3, PA2);
ODriveArduino odrive(Serial2);

// Global vars.
double y_0 = 0.51;
double y_1 = 0.58;
bool h_cal_flag = true, enc_inverted = false, dir, is_rising=1, is_falling=0, pres_cal_fail = 0, reset_flag = 0;
uint32_t next_motor_update = 0, next_speed_update = 0, next_dir_change, next_screen_update, next_params_update, next_serial_update, next_pres_cal, next_sensor_update, next_plot_update, cycle, next_error_update;
double pos_to_vel, flow, volume, volume_in, volume_out, pip, peep, max_ins_flow, max_exp_flow;
bool plot_enable = 1, sensor_update_enable = 1, params_update_enable = 1, motor_control_enable = 1, odrive_errors_update_enable = 0;
double k_vol = 0.3;
int32_t zero_position = 0;
uint32_t odrive_errors[5];
int32_t curr_enc_pos;

RotaryEncoder encoder(PIN_ENCODER_A, PIN_ENCODER_B, PB1);

// Global objects.
LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);  // Set the LCD I2C address
Adafruit_ADS1115 ads;

// Prototypes
void BlinkLED();
void calibrate();
void encoderISR();
void limitswitchISR();
void lcd_update(LcdVals *lcd_vals);
void home();
void reset_vals();
void hard_calibrate();
int8_t read_errors(HardwareSerial*, ODriveArduino*, uint8_t, uint32_t*, uint32_t );
uint8_t get_errors(HardwareSerial*, uint32_t*, uint32_t);
void print_errors_string(char serial_buff[], uint32_t serial_size, uint32_t odrive_errors[], uint32_t error_size);
void restart_odrive(ODriveArduino* odrive, HardwareSerial* Serial2);
void odrive_cal(uint8_t motor);
void handle_odrive_error();
void go_home();
void move_arm_to(int32_t);


void setup()
{
  motor_vals.serial_out = &Serial2;
  motor_vals.odrive = &odrive;
  motor_vals.axis = 0;
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_LIMIT_SWITCH, INPUT_PULLDOWN);
  pinMode(PIN_PWM, OUTPUT_OPEN_DRAIN);
  pinMode(PIN_DIR, OUTPUT_OPEN_DRAIN);
  digitalWrite(PIN_PWM, 0);
  // encoder.begin();
  // attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoderISR, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), encoderISR, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(PIN_LIMIT_SWITCH), limitswitchISR, CHANGE);
  sys_state.limit_switch_state = !digitalRead(PIN_LIMIT_SWITCH);
  #ifdef SCREEN
  lcd.begin(16, 2, LCD_5x8DOTS);
  lcd.setCursor(0,0);
  #endif
  analogReadResolution(12);
  analogWriteFrequency(20000);
  Serial.begin(115200);
  Serial.println("Booting up...");
  Serial2.begin(115200);
  ads.setGain(GAIN_SIXTEEN);
  ads.begin();
  ads.setSPS(ADS1115_DR_860SPS); 
  // calibrate_pressure_sensor(&pres_0); // TODO: Review this
  calibrate_pressure_sensor(&pres_1);
  pres_0.openpressure = 253;
  Serial.println("Done!");
  // pres_1.openpressure = 13;
}

void loop() 
{
  BlinkLED();
  if(!sys_state.odrive_cal_flag) odrive_cal(0);
  if(!sys_state.cal_flag && sys_state.odrive_cal_flag) calibrate();

  if (odrive_errors_update_enable && millis() > next_error_update)
  {

    if(read_errors(&Serial2, &odrive, 0, odrive_errors, sizeof(odrive_errors)))
    {
      handle_odrive_error();
    }
    next_error_update = millis() + ERROR_UPDATE_DELAY;
  }

  if (sensor_update_enable && millis() > next_sensor_update)
  {
    read_pressure(&pres_0, &current_flow);
    read_pressure_2(&pres_1, &current_flow);
    calculate_flow_oplate(&current_flow);
    calculate_flow_state(&step, &sys_state, &odrive, &curve_vals, &current_flow);
    //flow_controller(&step, &sys_state, &control_vals, &curve_vals, &new_vals, &current_flow);
    next_sensor_update = millis() + SENSOR_UDPATE_DELAY;
  }

  if(params_update_enable && (millis()>next_params_update || sys_state.params_change_flag))
  {
    update_params(&step, &sys_state, &curve_vals, &new_vals);
    next_params_update = millis() + PARAMS_UPDATE_DELAY;
  }

  if(sys_state.plot_enable && millis()>next_plot_update)
  {
    plot_data(&step, &curve_vals, &motor_vals, &current_flow);
    next_plot_update = millis() + PLOT_UPDATE_DELAY;
  }

  if(motor_control_enable && millis()>next_motor_update && (sys_state.play_state == PLAY || sys_state.play_state==PAUSE) && sys_state.cal_flag)
  {
    calc_step(&step, &sys_state, &curve_vals);
    read_motor(&motor_vals, &odrive);
    generate_curve(&step, &sys_state, &motor_vals, &curve_vals);
    mimo_control(&motor_vals, &control_vals, &step);
    filter_motor(&motor_vals);
    execute_motor(&sys_state, &motor_vals);
    next_motor_update = millis() + MOTOR_UPDATE_DELAY;
  }

  if(sys_state.play_state == STOP && !reset_flag)
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
      sys_state.play_state = PAUSE;
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
      parse_params(serial_buf, (sizeof(serial_buf)/sizeof(char)),  &sys_state, &control_vals, &curve_vals, &new_vals);
      next_serial_update = millis() + SERIAL_UPDATE_DELAY;
    }
    else next_serial_update = millis() + SERIAL_UPDATE_DELAY;
  }
}

void reset_vals()
{
  sys_state.restart_step_flag = true;
  volume = 0;
}

void home()
{
  Serial.println(encoder.getPosition()<<1);
  if(encoder.getPosition()<<1 == zero_position) reset_flag = 1;
  else if(encoder.getPosition()<<1 > zero_position) motor_write(&motor_vals, -50);
  else if (encoder.getPosition()<<1 < zero_position) motor_write(&motor_vals, 50);
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

void limitswitchISR() // TODO: Find better debouncing method
{
  if(!digitalRead(PIN_LIMIT_SWITCH))
  {
    motor_write(&motor_vals, 0);
    sys_state.limit_switch_state = LIMIT_SWITCH_LOGIC;
  }
  else 
  {
    sys_state.limit_switch_state = !LIMIT_SWITCH_LOGIC;
  }
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

//TODO add check for limit switch trigger
void odrive_cal(uint8_t motor) // Runs the full calibration routine for ODrive motor
{
  static uint8_t state = 0;
  static uint32_t odrive_errors[5];
  static uint8_t restart_retries=1;
  static uint8_t cal_retries=1;
  char serial_buf[100];

  switch (state)
  {
  case 0: //Init calibration
  {
    sys_state.play_state = ODRIVE_CAL;
    sys_state.cal_flag = false; //Makes sure calibration is not running while the ODrive is calibrating
    odrive_errors_update_enable = 0;
    Serial.println("[LOG]Starting ODrive calibration routine."); //TODO Make a logging function.
    state++;
    break;
  }
  case 1: //Check for init errors
  {
    Serial.println("[LOG]Checking ODrive errors...");
    read_errors(&Serial2, &odrive, 0, odrive_errors, sizeof(odrive_errors));
    if(get_errors(&Serial1, odrive_errors, sizeof(odrive_errors)))
    {
      Serial.print("[WARN]ODrive error(s) detected: "); //TODO Make a warning function
      print_errors_string(serial_buf, sizeof(serial_buf), odrive_errors, sizeof(odrive_errors));
      Serial.print(serial_buf);
      if(restart_retries <= MAX_RESTART_RETRIES)
      {
        snprintf(serial_buf, sizeof(serial_buf), "[LOG]Restarting ODrive... Try %d/%d", restart_retries++, MAX_RESTART_RETRIES);
        Serial.println(serial_buf);
        Serial.println("[LOG]Restarting ODrive...");
        restart_odrive(&odrive, &Serial2); //TODO Blocking!
        break;
      }
      else
      {
        snprintf(serial_buf, sizeof(serial_buf), "[CRITICAL]Errors still detected after %d retries. Entering failmode", restart_retries); //TODO make a critial error function
        Serial.println(serial_buf);
        sys_state.play_state=FAIL;
        break;
      }
    }
    else
    {
      Serial.println("[LOG]No ODrive errors detected.");
      restart_retries = 0;
      state++;
      break;
    }
  }
  case 2: //Start cal
  {
    Serial.println("[LOG]Starting full calibration sequence...");
    int requested_state = ODriveArduino::AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
    if(odrive.run_state(motor, requested_state, true)) //TODO Blocking!
    {
      Serial.println("[WARN]ODrive timed out, restarting..."); //TODO Max number of retries?
      restart_odrive(&odrive, &Serial2);
      state--;
      break;
    }
    delay(5000); //TODO Verify calibration completion.
    read_errors(&Serial2, &odrive, 0, odrive_errors, sizeof(odrive_errors));
    if(get_errors(&Serial1, odrive_errors, sizeof(odrive_errors)))
    {
      Serial.println("[WARN]ODrive error(s) detected while calibrating: "); //TODO Make a warning function
      print_errors_string(serial_buf, sizeof(serial_buf), odrive_errors, sizeof(odrive_errors));
      Serial.println(serial_buf);
      if(cal_retries <= MAX_ODRIVE_CAL_RETRIES)
      {
        snprintf(serial_buf, sizeof(serial_buf), "[LOG]Retrying ODrive calibration... Try %d/%d", cal_retries++, MAX_ODRIVE_CAL_RETRIES);
        Serial.println(serial_buf);
        Serial.println("[LOG]Restarting ODrive...");
        restart_odrive(&odrive, &Serial2); //TODO Blocking!
        state--;
        break;
      }
      else
      {
        snprintf(serial_buf, sizeof(serial_buf), "[CRITICAL]Errors still detected after %d calibration retries, entering fail mode", restart_retries); //TODO make a critial error function
        Serial.println(serial_buf);
        sys_state.play_state=FAIL;
        state = 0;
        break;
      }
    }
    else
    {
      Serial.println("[LOG]No ODrive errors detected while calibrating.");
      cal_retries = 0;
      int requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
      odrive.run_state(motor, requested_state, true);
    }
    state = 0;
    sys_state.odrive_cal_flag = true;
    odrive_errors_update_enable = true;
    sys_state.play_state = PAUSE; //Ready to start.
    break;
  }
  case 3:
  {
  }
  }
}

void handle_odrive_error() //TODO add critical fail alarm
{
  Serial.println("[CRITICAL]ODrive error during operation. Entering failmode.");
  sys_state.play_state=FAIL;
}

void go_home()
{
  static bool init = 0;
  if(!init)
  {
    sys_state.play_state = CAL;
    Serial.println("[LOG]Homing arm...");
    init = !init;
  }
  if(!digitalRead(PIN_LIMIT_SWITCH)) //TODO timeout.
  {
    odrive_speed_write(&motor_vals, -HOMING_SPEED);
  }
  else
  {
    odrive_speed_write(&motor_vals, 0);
    Serial.println("[LOG]Arm successfully homed.");
    sys_state.homed=true;
    init = 0;
  }
}

void move_arm_to(uint8_t axis, int32_t pos)
{
  static bool init = 0;
  char serial_buf[50];
  if(!init)
  {
    sys_state.play_state = CAL;
    Serial.println("[LOG]Moving arm to set position...");
    init = !init;
  }
  if(odrive_read_encoder(&Serial2, &odrive, 0) > pos+HOME_DZ) //TODO timeout.
  {
    odrive_speed_write(&motor_vals, -HOMING_SPEED);
  }
  else if(odrive_read_encoder(&Serial2, &odrive, 0) < pos-HOME_DZ) //TODO timeout.
  {
    odrive_speed_write(&motor_vals, HOMING_SPEED);
  }
  else
  {
    odrive_speed_write(&motor_vals, 0);
    Serial.println("[LOG]Arm successfully reached postion successfully.");
    sys_state.homed=true;
    init = 0;
  }
}

void calibrate()
{
  static uint8_t state = 0;
  char serial_buf[100];
  static int32_t zero, closed, home_pos;

  switch (state)
  {
  case 0:
  {
    sys_state.play_state = CAL;
    sys_state.zeroed = false;
    sys_state.homed = false;
    int requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
    odrive.run_state(0, requested_state, true);
    Serial.println("[LOG]Starting arm calibration routine.");
    Serial.println("[LOG]Finding open limit.");
    state++;
    break;
  }
  case 1:
  {
    if(!sys_state.homed)go_home();
    else state++;
    break;
  }
  case 2:
  {
    // encoder.setPosition(0);
    zero = odrive_read_encoder(&Serial2, &odrive, 0);
    snprintf(serial_buf, sizeof(serial_buf), "[LOG]Open limit saved at: %d", zero);
    Serial.println(serial_buf);
    Serial.println("[LOG]Place arm in closed position. Send ZERO when ready...");
    int requested_state = ODriveArduino::AXIS_STATE_IDLE;
    odrive.run_state(0, requested_state, true);
    state++;
    break;
  }
  case 3:
  {
    if(sys_state.zeroed)
    {
      closed = odrive_read_encoder(&Serial2, &odrive, ARM_AXIS);
      snprintf(serial_buf, sizeof(serial_buf), "[LOG]Closed limit saved at: %d", closed);
      Serial.println(serial_buf);
      snprintf(serial_buf, sizeof(serial_buf), "[LOG]Total range is: %ld", (int32_t)(CLICKS_TO_RAD*RAD_TO_DEG*((double)(closed-zero))));
      Serial.println(serial_buf);
      home_pos = (closed)-(int32_t)(AMBU_OPEN_ANGLE*0.017453293*RAD_TO_CLICK); //TODO handle case when range is too low
      snprintf(serial_buf, sizeof(serial_buf), "[LOG]Goal: %ld Current: %ld", home_pos, odrive_read_encoder(&Serial2, &odrive, ARM_AXIS));
      Serial.println(serial_buf);
      Serial.println("[LOG]Going to open position");
      sys_state.homed = false;
      int requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
      odrive.run_state(0, requested_state, true);
      state++;
      // sys_state.cal_flag=true;
      // state = 0;
    }
    break;
  }
  case 4:
  {
    // snprintf(serial_buf, sizeof(serial_buf), "[LOG]Goal: %ld Current: %ld", ((closed-zero)-(int32_t)(AMBU_OPEN_ANGLE*227.5555)), encoder.getPosition());
    // Serial.println(serial_buf);
    if(!sys_state.homed)move_arm_to(ARM_AXIS, home_pos);
    else state++;
    break;
  }
  case 5:
  {
    sys_state.cal_flag=true;
    state = 0;
    Serial.println("[LOG]Arm successfully calibrated.");
    Serial.println(odrive_read_encoder(&Serial2, &odrive, ARM_AXIS));
    break;
  }
  default:
    break;
  }
}

int8_t read_errors(HardwareSerial* Serial2, ODriveArduino* odrive, uint8_t motor_id, uint32_t odrive_errors[], uint32_t size)
{
  char serial_buff[40];
  if (size != sizeof(uint32_t)*5 || motor_id > MAX_MOTOR_NUM) return -1;
  snprintf(serial_buff, sizeof(serial_buff), "r axis%d.error", motor_id);
  Serial2->println(serial_buff);
  odrive_errors[0] = odrive->readInt();
  snprintf(serial_buff, sizeof(serial_buff), "r axis%d.motor.error", motor_id);
  Serial2->println(serial_buff);
  odrive_errors[1] = odrive->readInt();
  snprintf(serial_buff, sizeof(serial_buff), "r axis%d.controller.error", motor_id);
  Serial2->println(serial_buff);
  odrive_errors[2] = odrive->readInt();
  snprintf(serial_buff, sizeof(serial_buff), "r axis%d.encoder.error", motor_id);
  Serial2->println(serial_buff);
  odrive_errors[3] = odrive->readInt();
  snprintf(serial_buff, sizeof(serial_buff), "r axis%d.sensorless_estimator.error", motor_id);
  Serial2->println(serial_buff);
  odrive_errors[4] = odrive->readInt();
  return 0;
}

void restart_odrive(ODriveArduino* odrive, HardwareSerial* Serial2)
{
  Serial2->println("sr");
  delay(10); //TODO No millis!
  int requested_state = ODriveArduino::AXIS_STATE_IDLE;
  odrive->run_state(0, requested_state, true); //TODO Blocking!
}

uint8_t get_errors(HardwareSerial *Serial1, uint32_t odrive_errors[], uint32_t size)
{
  uint8_t return_code = 0;
  if (size != sizeof(uint32_t)*5) return 0xFF;
  if(odrive_errors[0])
  {
    return_code += 1;
    if(odrive_errors[1])
    {
      return_code += 1<<1;
    }
    if(odrive_errors[2])
    {
      return_code += 1<<2;
    }
    if(odrive_errors[3])
    {
      return_code += 1<<3;
    }
    if(odrive_errors[4])
    {
      return_code += 1<<4;
    }
    return 1;
  }
  else
  {
   return return_code;
  }
}

void print_errors_string(char serial_buf[], uint32_t serial_size, uint32_t odrive_errors[], uint32_t error_size)
{
  if (error_size != sizeof(uint32_t)*5) return;
  snprintf(serial_buf, serial_size, "Axis error: %ld\nMotor error: %ld\nController error: %ld\nEncoder error: %ld\nSensorless Estimator error: %ld", odrive_errors[0], odrive_errors[1], odrive_errors[2], odrive_errors[3], odrive_errors[4]);
}