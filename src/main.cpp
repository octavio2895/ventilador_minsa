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
#define PLOT_UPDATE_DELAY     5

// Parameter
#define MAX_ADC_RESOLUTION    16


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
uint32_t next_motor_update = 0, next_speed_update = 0, next_dir_change, next_screen_update, next_params_update, next_serial_update, next_pres_cal, next_sensor_update, next_plot_update, cycle;
double pos_to_vel, flow, volume, volume_in, volume_out, pip, peep, max_ins_flow, max_exp_flow;
bool plot_enable = 1, sensor_update_enable = 1, params_update_enable = 1, motor_control_enable = 1;
double k_vol = 0.3;
int32_t zero_position = 0;

RotaryEncoder encoder(PIN_ENCODER_B, PIN_ENCODER_A, PB1);

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
void get_errors(HardwareSerial*, ODriveArduino*, uint32_t*, uint32_t );
uint8_t print_errors(HardwareSerial*, uint32_t*, uint32_t);


void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_LIMIT_SWITCH, INPUT_PULLUP);
  pinMode(PIN_PWM, OUTPUT_OPEN_DRAIN);
  pinMode(PIN_DIR, OUTPUT_OPEN_DRAIN);
  digitalWrite(PIN_PWM, 0);
  encoder.begin();
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_LIMIT_SWITCH), limitswitchISR, CHANGE);
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
  if(!sys_state.cal_flag) calibrate();
  if(!h_cal_flag) hard_calibrate();

  if (sensor_update_enable && millis() > next_sensor_update)
  {
    read_pressure(&pres_0, &current_flow);
    read_pressure_2(&pres_1, &current_flow);
    calculate_flow_oplate(&current_flow);
    calculate_flow_state(&step, &sys_state, &encoder, &curve_vals, &current_flow);
    flow_controller(&step, &sys_state, &control_vals, &curve_vals, &new_vals, &current_flow);
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

  if(motor_control_enable && millis()>next_motor_update && sys_state.play_state != STOP && sys_state.cal_flag)
  {
    calc_step(&step, &sys_state, &curve_vals);
    read_motor(&motor_vals, &encoder);
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
    sys_state.limit_switch_state = 1;
  }
  else 
  {
    sys_state.limit_switch_state = 0;
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

void calibrate()
{
  static uint8_t state = 0;
  static uint16_t stopped_millis = 0;
  static uint32_t odrive_errors[5];

  switch (state)
  {
  case 0:
  {
    sys_state.play_state = PAUSE;
    Serial.println("Comunicating with ODrive...");
    state++;
    break;
  }
  case 1:
  {
    Serial.println("Calibrating motor...");
    delay(10);
    get_errors(&Serial2, &odrive, odrive_errors, sizeof(odrive_errors));
    Serial.println(print_errors(&Serial1, odrive_errors, sizeof(odrive_errors)));
    int requested_state = ODriveArduino::AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
    odrive.run_state(1, requested_state, true);
    get_errors(&Serial2, &odrive, odrive_errors, sizeof(odrive_errors));
    Serial.println(print_errors(&Serial1, odrive_errors, sizeof(odrive_errors)));
    state++;
    break;
  }
  case 2:
  {
    Serial.println(" Done!");
    Serial.println(encoder.getPosition()<<1);
    encoder.setPosition(0);
    zero_position = (encoder.getPosition()<<1) + ZERO_OFFSET;
    Serial.println(zero_position);
    stopped_millis = millis() + 5000;
    state++;
    calibrate_pressure_sensor(&pres_0);
    break;
  }
  case 3:
  {
    if(millis() > stopped_millis) 
    {
      Serial.print("Going to zero...");
      state++;
    }
    break;
  }
  case 4:
  {
    if(((encoder.getPosition()<<1) - zero_position) < 0)
    {
      // Serial.println((encoder.getPosition()<<1) - zero_position);
      motor_write(&motor_vals, MIN_VEL);
    }
    else
    {
      state++;
      motor_write(&motor_vals, 0);
    }
    break;
  }
  case 5:
  {
    Serial.println(" Done!");
    sys_state.cal_flag = 1;
    state = 0;
    break;
  }
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
    sys_state.play_state = PAUSE;
    Serial.println("Performing hard calibration...");
    state++;
    break;
  case 1:
    if(digitalRead(PIN_LIMIT_SWITCH))
    {
      motor_write(&motor_vals, -MIN_VEL);
    }
    else
    {
      state++;
      motor_write(&motor_vals, 0);
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
  // case 4:
  // {
  //   uint32_t current_adc = analogRead(PIN_CURRENT_SENSE);
  //   if(current_adc < CURRENT_COLLISION_THRESHOLD)
  //   {
  //     Serial.println((encoder.getPosition()<<1) - zero_position);
  //     Serial.println(current_adc);
  //     motor_write(MIN_VEL);
  //   }
  //   else
  //   {
  //     motor_write(0);
  //     state++;
  //     Serial.print("Upper limit: ");
  //     Serial.println(encoder.getPosition());
  //     zero_position = encoder.getPosition() - AMBU_OPEN_ANGLE*DEG_TO_RAD*RAD_TO_CLICK;
  //     Serial.println("Going to zero...");
  //   }
  //   break;
  // }
  case 5:
    if(((encoder.getPosition()<<1) - zero_position) > 0)
    {
      Serial.println((encoder.getPosition()<<1) - zero_position);
      motor_write(&motor_vals, -MIN_VEL);
    }
    else
    {
      state++;
      motor_write(&motor_vals, 0);
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

void get_errors(HardwareSerial* Serial2, ODriveArduino* odrive, uint32_t odrive_errors[], uint32_t size)
{
  if (size != sizeof(uint32_t)*5) return;
  Serial2->println("r axis1.error");
  odrive_errors[0] = odrive->readInt();
  Serial2->println("r axis1.motor.error");
  odrive_errors[1] = odrive->readInt();
  Serial2->println("r axis1.controller.error");
  odrive_errors[2] = odrive->readInt();
  Serial2->println("r axis1.encoder.error");
  odrive_errors[3] = odrive->readInt();
  Serial2->println("r axis1.sensorless_estimator.error");
  odrive_errors[4] = odrive->readInt();
}

uint8_t print_errors(HardwareSerial *Serial1, uint32_t odrive_errors[], uint32_t size)
{
  uint8_t return_code = 0;
  if (size != sizeof(uint32_t)*5) return 0xFF;
  if(odrive_errors[0])
  {
    return_code += 1;
    Serial->print("Errors detected: ");
    if(odrive_errors[1])
    {
      return_code += 1<<1;
      Serial->println("Motor error ");
    }
    if(odrive_errors[2])
    {
      return_code += 1<<2;
      Serial->println("Controller error ");
    }
    if(odrive_errors[3])
    {
      return_code += 1<<3;
      Serial->println("Encoder error ");
    }
    if(odrive_errors[4])
    {
      return_code += 1<<4;
      Serial->println("Sensorless estimator error ");
    }
    return return_code;
  }
  else
  {
    Serial->println("No errors detected");
    return return_code;
  }
}