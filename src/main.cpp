#include <Arduino.h>
// #include <Encoder.h>
#include <RotaryEncoder.h>
#include <PID_v1.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <SerialTransfer.h>


// Pin definitions.
#define PIN_ENCODER_A PA7
#define PIN_ENCODER_B PA6
#define PIN_DIR_A PA4
#define PIN_DIR_B PA5
#define PIN_PWM PA3
#define PIN_LIMIT_SWITCH PA8
#define PIN_INVERT PB10
#define PIN_SERIAL PA0 // UPDATE kp de 0.01 - 0.25
#define PIN_REGULADOR_RANGO PA1 //UPDATE ANGULO DE OPERACION
#define PIN_REGULADOR_TIEMPO PA2 //


// Physical constraints.
#define ROTARYMIN 0
#define ROTARYMAX 200
#define ROTARYINITIAL 0
#define ENCODER_CPR 2000
#define MOTOR_DZ 0 //165

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
#define K2            8
#define K3            120
#define K4            8
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


//1076 & 79

// Structs and Enums
enum direction{FORWARD, BACKWARD, BRAKE, BRAKE2};
struct DATA {
  bool blinker;
  float humidity;
  float temperature;
  float heatIndex;
} transfer_data;

struct ControlVals {
  //Control encoder
double control_kp = 0.1;
uint16_t control_tiempo = 2000, control_rango = 75;

}control_vals;

// Global vars.
double motor_position, motor_angular_position, motor_velocity, motor_acceleration, motor_output, motor_target, target_position, target_velocity, kp = 1150, ki = 0/*.0005*/, kd = 0;
bool cal_flag = false, enc_inverted = false, dir, is_rising=1, is_falling=0;
uint16_t zero_position = 0;

uint32_t next_motor_update = 0, next_speed_update = 0, next_dir_change, next_screen_update, next_kp_update;
float const_vel_time_rise, const_vel_time_fall;
float theta_dot_max_rise,theta_dot_max_fall, print_curr_step, print_m_target,print_m_vel,print_pos;
uint32_t volatile start_pulse, end_pulse, pulse_width;
bool volatile signal;
double pos_to_vel;



// Global objects.
RotaryEncoder encoder(PIN_ENCODER_A, PIN_ENCODER_B, PB0);
LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);  // Set the LCD I2C address
PID motor(&motor_velocity, &motor_output, &motor_target, kp, ki, kd, DIRECT); //Input, Output, and Setpoint.  Initial tuning parameters are also set here.
//PID motor(&motor_position, &motor_output, &motor_target, kp, ki, kd, DIRECT);
Servo myservo;
SerialTransfer my_transfer;
HardwareSerial Serial2(PIN_SERIAL_TRANSFER_RX, PIN_SERIAL_TRANSFER_TX);

// Last known rotary position.
// int lastPos = 0;
// uint32_t delayBlink = 15096000;
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
void change_control_values(ControlVals *vals);
void BlinkLED();
void calibrate();
void motor_set_dir(double);
void motor_write(double);
int16_t calculate_position();
double calculate_angular_velocity();
int16_t calculate_angular_acceleration();
void encoderISR();
int16_t deg_to_clicks(double deg);
double clicks_to_rad(int16_t clicks);
float arr_average(float *arr, uint16_t size);
void velocity_control();
void mimo_control();
double get_target_position(uint32_t);
double get_target_velocity(uint32_t);
double deg_to_rad(double);
void read_control();
void pulse_interrupt();


void setup()
{


  pinMode(PIN_DIR_A, OUTPUT);
  pinMode(PIN_DIR_B, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_INVERT, INPUT);
  pinMode(PIN_POT_KP, INPUT_ANALOG);
  pinMode(PIN_POT_RANGE, INPUT_ANALOG);
  pinMode(PIN_POT_PERIOD, INPUT_ANALOG);


  pinMode(PIN_SERIAL, INPUT_PULLDOWN);
  pinMode(PIN_REGULADOR_RANGO, INPUT_ANALOG);
  pinMode(PIN_REGULADOR_TIEMPO, INPUT_ANALOG);


  if(digitalRead(PIN_INVERT)) RotaryEncoder encoder(PIN_ENCODER_B, PIN_ENCODER_A, PB0);

  encoder.begin();
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A),  encoderISR,       CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B),  encoderISR,       CHANGE);
  //attachInterrupt(digitalPinToInterrupt(PIN_SERIAL),  pulse_interrupt, CHANGE);

  motor.SetMode(AUTOMATIC);
  motor.SetOutputLimits(-500,500);
  lcd.begin(16, 2, LCD_5x8DOTS);
  lcd.setCursor(0,0);

  // lcd.print(F("Encoder: "));
  //analogWrite(PIN_PWM, 255);
  analogWriteFrequency(1000);

  digitalWrite(PIN_DIR_A, LOW);
  digitalWrite(PIN_DIR_B, HIGH);

  Serial.begin(115200);
  //Serial.println("Booting up!");


  myservo.attach(PIN_PWM, 1000, 2000);

  // myservo.writeMicroseconds(1400);
  // delay(1000);
  // myservo.writeMicroseconds(1600);
  // delay(1000);
  // currentTime = millis();

  const_vel_time_rise = RISE_TIME - 2*ACC_TIME;
  const_vel_time_fall = FALL_TIME - 2*ACC_TIME;
  theta_dot_max_rise = DEGREES/(const_vel_time_rise+ACC_TIME);
  theta_dot_max_fall = 36/10;//DEGREES/(const_vel_time_fall+ACC_TIME);


} 

void loop() 
{
  BlinkLED();

  // if(!cal_flag) calibrate();

  
  
  // if(millis()>next_dir_change)
  // {
  //   if (dir)
  //   {
  //     //motor_target = vals->control_rango; //100 A -100 ES EL ANGULO 0 - 1024 MAPEAR DE 0 A 200
  //     dir = !dir;
  //     is_falling = 0;
  //     is_rising = 1;
  //     next_dir_change = millis() + RISE_TIME;
  //   }
  //   else
  //   {
  //     //motor_target = -vals->control_rango;
  //     dir = !dir;
  //     is_rising = 0;
  //     is_falling = 1;
  //     next_dir_change = millis() + FALL_TIME;
  //   }
  //   // next_dir_change = millis() + vals->control_tiempo; // TERCER POTENCIOOMETRO DE 2 A 10 SEG
  // }



  if(millis()>next_motor_update)
  {

    //motor_write(95);
    mimo_control();
    //read_control();
    //velocity_control();

    next_motor_update = millis() + MOTOR_UPDATE_DELAY;
  }


  // if(millis()>next_speed_update)
  // {
  //   print_m_vel = motor_velocity*1000;
  //   motor_angular_position = (double) clicks_to_rad(motor_position);
  //   motor_velocity = (double) calculate_angular_velocity();
    
    
  //   next_speed_update = millis() + MOTOR_SPEED_DELAY;
  // }
  //LCD
  if (millis()>next_screen_update) 
  {
    lcd.clear();
    lcd.print("Enc:");
    // lcd.setCursor(9,0);

    lcd.print(encoder.getPosition());
    lcd.print(" ST:");
    lcd.print((int)print_curr_step);
    // lcd.print("pwm");
    // lcd.print(pwm);
    lcd.setCursor(0,1);
    lcd.print("O:");
    lcd.print(motor_output);
    lcd.print("T:");
    lcd.print(print_m_target);

    next_screen_update = millis() + 500;
  }
  if (millis()>next_kp_update)
  {

    change_control_values(&control_vals);
    next_kp_update = millis() + 1000;
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
  // }Merge branch 'master' of https://github.com/butuq/stPos != newPos) 
  // {
  //   lcd.setCursor(0,9);
  //   lcd.print(newPos);
  //   lastPos = newPos;
  // }
    //Serial.println("Target Velocity: ");
    //Serial.print(print_m_target);encoder.getPosition()
    // Serial.print(" ");
    // Serial.print(print_m_target);
    // Serial.print(" ");
    // //Serial.println("Real Velocity: ");
    // Serial.print(motor_output);
    // Serial.print(" ");
    // // Serial.p+ deg_to_rad(DEG_P_SEG)rint(print_pos);
    // // Serial.print(" ");
    // Serial.println(print_m_vel);

}

void read_control()
{
  int16_t encoder_val = encoder.getPosition();
  static uint64_t pulse_control,i,x,y;
  // motor_position = (double) calculate_position();   //Position
  // motor_angular_position = (double) clicks_to_rad(motor_position);

  //pulse_control = pulseIn(PIN_SERIAL,HIGH);
  y=y+10;
  i++;
  x=x+5;
  //Serial Print
  //Serial.println(millis());
  //Serial.print(",");
  Serial.println(encoder_val);
  //Serial.print("x");
  //Serial.println(pulse_width);
}

void mimo_control()
{
  static uint32_t next_target_update = 0;
  static double error_position, prx1,prx2,prx3;
  static double error_velocity, prev_velocity;
  static double motor_pwm, motor_volts, motor_pwm_filter, prev_pwm,motor_v_unf;
  static uint32_t initial_millis = millis();
  uint32_t current_step = (millis()-initial_millis)%(RISE_TIME/*+DEAD_TIME+FALL_TIME*/+WAITING_TIME);

  //Define Target values
  if (millis()>next_target_update)
  {
    target_position = 1*get_target_position(current_step);
    target_velocity = 1*get_target_velocity(current_step);
    next_target_update = millis() + TARGET_UPDATE_DELAY;
  }
// target_position = 10*get_target_position(current_step);
// target_velocity = get_target_velocity(current_step);
  //Read Real values
  motor_position = (double) calculate_position();   //Position
  motor_angular_position = (double) clicks_to_rad(motor_position);
  motor_v_unf = (double) calculate_angular_velocity();
  motor_velocity = (double) ((motor_v_unf + prev_velocity * FILTER) / (FILTER + 1));  //Velocity
  prev_velocity = motor_velocity;
  //Calculate error
  error_position = target_position - motor_angular_position;
  error_velocity = target_velocity - motor_velocity;

  //Error protection
  if (error_position > 0.8)
  {
    error_position = 0.8;
  }
  else if (error_position < -0.8)
  {
    error_position = -0.8;
  }
  
  //Motor Output
  if(current_step < SECOND_TIME)
  {
    motor_volts = K1 * error_position + K2 * error_velocity; //V/rad
  }
  else if (current_step < THIRD_TIME)
  {
    motor_volts = K3 * error_position + K4 * error_velocity; //V/rad
  }
  else
  {
    motor_volts = K1_W * error_position + K2_W * error_velocity;
  }

  
  
  motor_pwm = map(motor_volts, 0, 12, 0, 500);
  motor_pwm_filter = (motor_pwm + prev_pwm * FILTER_PWM) / (FILTER_PWM + 1);
  prev_pwm = motor_pwm_filter;

  
  if(motor_pwm_filter > 500)
  {
    motor_pwm_filter = 500;
  }
  else if (motor_pwm_filter < -500)
  {
    motor_pwm_filter = -500;
  }

  // if (current_step > RISE_TIME)
  // {
  //   motor_pwm_filter = -100;
  //   if(motor_angular_position < 0)
  //   {
  //     motor_pwm_filter = 40;
  //   }
  // }


  // if(motor_angular_position < 0)
  // {
  //   motor_pwm = 0;
  // }
  motor_write(motor_pwm_filter); //PWM


  
  //  Serial.println(millis());
  prx1= 100*error_position;
  prx2 = 100*motor_angular_position;
  prx3 = 100*target_position;
   Serial.print(prx1);
   Serial.print(" ");
   Serial.print(prx2);
   Serial.print(" ");
   Serial.print(prx3);
   Serial.print(" ");
   Serial.println(motor_volts,5);
  //  Serial.print(" ");
  //  Serial.print(error_velocity);
  //  Serial.print(" ");
  //  Serial.print(motor_velocity);
  //  Serial.print(" ");
  //  Serial.println(target_velocity);
  //  Serial.print(" ");
  //  Serial.print(target_position,5);
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
void velocity_control()
{
    motor_position = (double) calculate_position();
    // motor_angular_position = (double) clicks_to_rad(motor_position);
    // motor_velocity = (double) calculate_angular_velocity();
    // motor_acceleration = (double) calculate_angular_acceleration();
    static float initial_pos = -177;
  
    static uint32_t initial_millis = millis();
    
    float current_step = (millis()-initial_millis)%(RISE_TIME/*+DEAD_TIME+FALL_TIME/*+WAITING_TIME*/);
    print_curr_step = current_step;
    
    //Rising
    // if(motor_position < initial_po    acum_error += motor_target*MOTOR_UPDATE_DELAY;s && current_step > (ACC_TIME + const_vel_time_rise) && current_step <= RISE_TIME)
    // {
    //   motor_target = 0;
      
    // }
    /*else */
    if(current_step <= ACC_TIME)
    {
      if(is_falling)
      {
        
      }
      is_falling = 0;
      //is_rising = 1;
      motor_target = (theta_dot_max_rise/ACC_TIME)*(current_step);
    }
    else if (current_step > ACC_TIME && current_step <= ACC_TIME+const_vel_time_rise)
    {
      motor_target = theta_dot_max_rise;
    }
    else if (current_step > (ACC_TIME + const_vel_time_rise) && current_step <= RISE_TIME)
    {
      motor_target = -(theta_dot_max_rise/ACC_TIME)*(current_step-(ACC_TIME+const_vel_time_rise))+theta_dot_max_rise;
    }
    //Dead time
    else if ( current_step > RISE_TIME && current_step <= RISE_TIME+DEAD_TIME)
    {
      motor_target = 0;
    }    
    //Falling
    // else if (motor_position > 0)
    // { 
    //   motor_target = 0;
    // } 
    
    else if (current_step > RISE_TIME+DEAD_TIME && current_step <= (RISE_TIME+ACC_TIME+DEAD_TIME))
    {
      if(is_rising)
      {
        initial_pos = motor_position;
      }
      is_rising = 0;
      is_falling = 1;
      motor_target = -(theta_dot_max_fall/ACC_TIME)*(current_step-(RISE_TIME+DEAD_TIME));
    }
    else if (current_step > (RISE_TIME + DEAD_TIME + ACC_TIME) && current_step <= (RISE_TIME + DEAD_TIME + ACC_TIME + const_vel_time_fall))
    {
      motor_target = -theta_dot_max_fall;
    }
    else if (current_step > (RISE_TIME + DEAD_TIME + ACC_TIME + const_vel_time_fall) && current_step <= (RISE_TIME+DEAD_TIME+FALL_TIME))
    {
      motor_target = (theta_dot_max_fall/ACC_TIME)*(current_step-(RISE_TIME+DEAD_TIME+ACC_TIME+const_vel_time_fall))-theta_dot_max_fall;
    }
    else if (current_step > (RISE_TIME+DEAD_TIME+FALL_TIME) && current_step <= (RISE_TIME+DEAD_TIME+FALL_TIME+WAITING_TIME))
    {
      motor_target = 0;
    }
    //motor_target = (theta_dot_max_rise/ACC_TIME)*(current_step);

    print_m_target = motor_target*1000 ;//motor_target*1000;//motor_target*1000;
    
    print_pos = motor_angular_position;
    motor.Compute();
    motor_write(motor_output); //PWM
    // analogWrite(PIN_PWM, 256);
}
double get_target_position(uint32_t current_step)
{
  double static target_pos,prove;
  bool static init1 = 1, init2 = 1, init3 = 1;
  double static last_target;

  if(current_step <= FIRST_TIME)
  {
    init1 = 1;
    init2 = 1;
    init3 = 1;
    target_pos = 0.5*(current_step*current_step)/1000*(deg_to_rad(DEG_P_SEG))/FIRST_TIME;
  }
  else if (current_step <= SECOND_TIME)
  {
    if(init1)
    {
      last_target = target_pos;
      init1 = 0;
    }
    target_pos = deg_to_rad(DEG_P_SEG) * (current_step-FIRST_TIME)/1000 + last_target; 
  }
  else if (current_step <= THIRD_TIME)
  {
    if(init2)
    {
      last_target = target_pos;
      init2 = 0;
    }   
    target_pos = ((deg_to_rad(-DEG_P_SEG))/(THIRD_TIME-SECOND_TIME))*(0.5*(current_step-SECOND_TIME)*(current_step-SECOND_TIME)/1000-SECOND_TIME*(current_step-SECOND_TIME)/1000)+ deg_to_rad(DEG_P_SEG)*(current_step-SECOND_TIME)/1000 + last_target;
    //target_pos = 0.002828 * (pow(current_step-1300,0.5)+ 67.8823);
  }
  else
  {
    if(init3)
    {
      last_target = target_pos;
      pos_to_vel = last_target;
      init3 = 0;
    }   
    target_pos = (-last_target/WAITING_TIME)*(current_step-THIRD_TIME)+last_target;
  }
   //prove = target_pos*100;
  // Serial.print(last_target);
  // Serial.print(" ");
  //Serial.println(prove);
  return target_pos;
}
double get_target_velocity(uint32_t current_step)
{
  double target_vel;
  if(current_step <= FIRST_TIME)
  {
    target_vel = current_step*(deg_to_rad(DEG_P_SEG))/FIRST_TIME;
  }
  else if (current_step > FIRST_TIME && current_step <= SECOND_TIME)
  {
    target_vel = deg_to_rad(DEG_P_SEG);
  }
  else if (current_step > SECOND_TIME && current_step <= THIRD_TIME)
  {
    target_vel = ((deg_to_rad(-DEG_P_SEG))/(THIRD_TIME-SECOND_TIME))*(current_step-SECOND_TIME) + deg_to_rad(DEG_P_SEG);
  }
  else
  {
    target_vel = (-pos_to_vel/WAITING_TIME);
  }
  return target_vel;
}

void change_control_values(ControlVals *vals)
{
    // control_rango = map(analogRead(PIN_REGULADOR_RANGO),0,1024,0,50);
    // control_tiempo = map(analogRead(PIN_REGULADOR_TIEMPO),0,1024,2000,10000);
    // control_kp = ((double)map(analogRead(PIN_SERIAL),0,1024,100,2500))/10000;

    
    
    //motor.SetTunings(control_kp,0,0.015);

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
  signal = digitalRead(PIN_SERIAL);
  if(signal)
  {
    start_pulse = micros();
  }
  else if(signal = 0)
  {
    end_pulse = micros();
    pulse_width = end_pulse-start_pulse;
  }
}

int16_t calculate_position()
{
  int16_t pos = encoder.getPosition() - zero_position;
  return pos;
}

double calculate_angular_velocity()
{
  static double prev_angular_position;
  static uint32_t  old_millis;
  double velocity = 1000*(motor_angular_position - prev_angular_position)/(millis()- old_millis);
  prev_angular_position = motor_angular_position;
  old_millis = millis();
  
  return velocity;
}

int16_t calculate_angular_acceleration()
{
  static int16_t prev_velocity = 0;
  int16_t acceleration = (motor_velocity - prev_velocity)/MOTOR_UPDATE_DELAY;
  prev_velocity = motor_velocity;
  return acceleration;
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
// {

// }

