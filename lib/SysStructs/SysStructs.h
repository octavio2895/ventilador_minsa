#ifndef STRUCTS_LIB
#define STRUCTS_LIB

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <ODriveArduino.h>

#define CRC_POLYNOMIAL        (0x0131)
#define K1                    120//85
#define K2                    0.5 //4
#define K3                    300
#define K4                    2.5//4
#define K5                    500//4
#define K6                    .5//4
#define ACCEL_1               320
#define ACCEL_2               -20
#define ACCEL_3               -160
#define MAX_ACCEL             700
#define MAX_VOL               65
#define MIN_VOL               0
#define MAX_RR                40
#define MIN_RR                0
#define MAX_X                 4
#define MIN_X                 1
#define BREATH_PAUSE          0
#define GAIN_POINT_0          0.39
#define GAIN_POINT_1          0.8
#define ARM_AXIS              0

// #define USE_FLUTTER_PRINTS
#define INVERTED_SPEED

// TODO: Review why this works

struct CurveParams
{
  double rr = 15;
  double x = 1;
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
  double target_vol = .7;
  double a_t = 55;

};

struct PressureSensor
{
  uint8_t id;
  int16_t openpressure;
  double pressure_adc;
  double pressure;
};

struct FlowData
{
  double flow;
  double volume;
  double vti;
  double vte;
  double pip;
  double peep;
  double flow_ins_max;
  double flow_exp_max;
  double angle;
  double error;
  double pressure;
  double differential_pressure;
};

struct ControlVals 
{
  double kp[7] = {K1, K3, K5, K1, K1, K1, K1};
  double kv[7] = {K2, K4, K6, K2, K2, K2, K2};
  double kvol = 0.04;
};

struct MotorDynamics
{
  HardwareSerial* serial_out;
  ODriveArduino* odrive;
  int32_t lower_limit = 0;
  int32_t upper_limit;
  int32_t current_pos = 0;
  double current_ang_pos = 0;
  double target_pos;
  double current_vel;
  double target_vel;
  uint16_t output_range = 256;
  int32_t output;
  volatile uint32_t click_time = 1; // Avoid div /0
  volatile bool click_dir;
  int16_t dir_pin;
  int16_t pwm_pin;
  double motor_volts;
  int16_t axis;
};


enum Stages //Stages of respiration
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
  uint32_t cycle;
};

enum PlayStates
{
    STOP = 0,
    PAUSE = 1,
    PLAY = 2,
    CAL = 3,
    ODRIVE_CAL = 4,
    FAIL = 5
};

struct SysState
{
    PlayStates play_state = PAUSE;
    int32_t zero_pos = 0;
    bool cal_flag = 0;
    bool odrive_cal_flag = 0;
    bool plot_enable = 0;
    bool params_change_flag = 0;
    bool restart_step_flag = 0;
    bool limit_switch_state = 0;
    bool find_home = 0;
    bool homed = 0;
    bool zeroed = 0;
};


#endif