#include <Arduino.h>
// #include <Encoder.h>
#include <RotaryEncoder.h>
#include <PID_v1.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Pin definitions.
#define PIN_ENCODER_A PA7
#define PIN_ENCODER_B PA6
#define PIN_DIR_A PA4
#define PIN_DIR_B PA5
#define PIN_PWM PA3
#define PIN_LIMIT_SWITCH PA8
#define PIN_INVERT PA1
//222
// Physical constraints.
#define ROTARYMIN 0
#define ROTARYMAX 200
#define ROTARYINITIAL 0
#define ENCODER_CPR 2000
#define MOTOR_DZ 165

// Update rates.
#define MOTOR_UPDATE_DELAY 50
#define SCREEN_UPDATE_DELAY 100
#define BLINK_DELAY 500

// Parameter
#define CAL_PWM 10
#define CAL_DRIVE_BACK_ANG 5
#define CAL_DRIVE_ANG 0.5
#define CAL_DZ 0.5

// Structs and Enums
enum direction{FORWARD, BACKWARD, BRAKE, BRAKE2};

// Global vars.
double motor_position, motor_output, motor_target, kp = 0.1, ki = .0005, kd = 0.015;
bool cal_flag = false, enc_inverted = false, dir;
uint16_t zero_position = 0;
uint32_t next_motor_update = 0, next_dir_change, next_screen_update;
// Global objects.
RotaryEncoder encoder(PIN_ENCODER_A, PIN_ENCODER_B, PA0);
LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);  // Set the LCD I2C address
PID motor(&motor_position, &motor_output, &motor_target, kp, ki, kd, DIRECT);

// Last known rotary position.
// int lastPos = 0;
// uint32_t delayBlink = 1500;
// uint32_t currentTime;
// bool forward = true;

// u_int16_t refreshScreen = 100;
// int a = 5;
// bool test = true;
/*
void encoderISR() {
  encoder.readAB();
}

void encoderButtonISR() {
  encoder.readPushButton();
}*/
// void ISR(PCINT1_vect) {
//   encoder.tick(); // just call tick() to check the state.
// }

// Prototypes
void BlinkLED();
void calibrate();
void motor_set_dir(double);
void motor_write(double);
int16_t calculate_position();
void encoderISR();
int16_t deg_to_clicks(double deg);




void setup()
{
  pinMode(PIN_DIR_A, OUTPUT);
  pinMode(PIN_DIR_B, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_INVERT, INPUT);
  if(digitalRead(PIN_INVERT)) RotaryEncoder encoder(PIN_ENCODER_B, PIN_ENCODER_A, PA0);
  encoder.begin();
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A),  encoderISR,       CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B),  encoderISR,       CHANGE);

  motor.SetMode(AUTOMATIC);
  motor.SetOutputLimits(-(256-MOTOR_DZ), (256-MOTOR_DZ));
  
  lcd.begin(16, 2, LCD_5x8DOTS);
  lcd.setCursor(0,0);
  // lcd.print(F("Encoder: "));
  analogWrite(PIN_PWM, 255);
  analogWriteFrequency(1000);
  digitalWrite(PIN_DIR_A, LOW);
  digitalWrite(PIN_DIR_B, HIGH);

  // currentTime = millis();
} 

void loop() 
{
  // if(!cal_flag) calibrate();
  BlinkLED();

  if(millis()>next_dir_change)
  {
    if (dir)
    {
      motor_target = 100;
      dir = !dir;
    }
    else
    {
      motor_target = -100;
      dir = !dir;
    }
    next_dir_change = millis() + 10000;
  }

  if(millis()>next_motor_update)
  {
    motor_position = (double) calculate_position();
    motor.Compute();
    motor_set_dir(motor_output);
    motor_write(motor_output);
    // analogWrite(PIN_PWM, 256);
    next_motor_update = millis() + MOTOR_UPDATE_DELAY;
  }


  if (millis()>next_screen_update) 
  {
    lcd.clear();
    lcd.print("Encoder:");
    // lcd.setCursor(9,0);
    lcd.print(encoder.getPosition());
    lcd.setCursor(0,1);
    lcd.print("O:");
    lcd.print(motor_output);
    lcd.print("T:");
    lcd.print(motor_target);
    next_screen_update = millis() + 100;
  }

  // if (newPos < ROTARYMIN) 
  // {
  //   digitalWrite(pinMotor1, LOW);
  //   digitalWrite(pinMotor2, HIGH);
  // } 
  // else if (newPos > ROTARYMAX) 
  // {
  //   digitalWrite(pinMotor1,HIGH);
  //   digitalWrite(pinMotor2,LOW);
  // }

  // if (lastPos != newPos) 
  // {
  //   lcd.setCursor(0,9);
  //   lcd.print(newPos);
  //   lastPos = newPos;
  // } 
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

void calibrate()
{
  static int16_t prev_position = 0, init_position = 0;
  static bool init = true, init_on_limit, is_inverted, is_back, driving_back;
  static uint32_t millis_driving_back;

  if(init)
  {
    init_on_limit = digitalRead(PIN_LIMIT_SWITCH);
    init_position = encoder.getPosition();
    init = false;
  }
  

  if (init_on_limit && !is_back)
  {
    /* If the position is going in the oposite direction, 
    or the position is stuck withing a zone and some time has passed, and the motor is actively driving... */
    if (((encoder.getPosition() < (prev_position-1)) || ((encoder.getPosition() < (prev_position+5)) && (millis() > millis_driving_back + 1000))) && driving_back) 
    {
      enc_inverted = !enc_inverted;
      prev_position = encoder.getPosition();
    }
    if (encoder.getPosition() < (init_position + CAL_DRIVE_BACK_ANG) && !driving_back) 
    {
     motor_target = init_position + CAL_DRIVE_BACK_ANG;
     prev_position = encoder.getPosition();
     driving_back = true;
     millis_driving_back = millis();
    }
    else if (encoder.getPosition() >= (init_position + CAL_DRIVE_BACK_ANG) && driving_back)
    {
      driving_back = false;
      is_back = true;
    }
  }

  /* octavio se la come*/

  else
  {
    if (digitalRead(PIN_LIMIT_SWITCH))
    {
      zero_position = encoder.getPosition();
      cal_flag = true;
    }
    
    else
    {
      if (encoder.getPosition() > (prev_position+deg_to_clicks(CAL_DZ))) enc_inverted = !enc_inverted;
      prev_position = encoder.getPosition();
      motor_target = prev_position - CAL_DRIVE_ANG;
    }
  }
  return;
}

void motor_set_dir(double pwm)
{
  if(!enc_inverted)
  {
    if(pwm > 0)
    {
      digitalWrite(PIN_DIR_A, HIGH);
      digitalWrite(PIN_DIR_B, LOW);
    }
    else
    {
      digitalWrite(PIN_DIR_A, LOW);
      digitalWrite(PIN_DIR_B, HIGH);
    }
  }
  else
  {
    if(pwm > 0)
    {
      digitalWrite(PIN_DIR_A, LOW);
      digitalWrite(PIN_DIR_B, HIGH);
    }
    else
    {
      digitalWrite(PIN_DIR_A, HIGH);
      digitalWrite(PIN_DIR_B, LOW);
    }
  }
  return;
}

void motor_write(double pwm)
{
  if(pwm  >0) pwm = pwm + MOTOR_DZ;
  else if (pwm < 0) pwm = pwm - MOTOR_DZ;
  analogWrite(PIN_PWM, abs(pwm));
  return;
}

double clicks_to_deg(int16_t clicks)
{
  return((double)(((double)360/(double)ENCODER_CPR)*(double)clicks));
}

int16_t deg_to_clicks(double deg)
{
  return((int16_t)((deg/360)*ENCODER_CPR));
}
