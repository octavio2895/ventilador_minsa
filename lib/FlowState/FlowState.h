#ifndef FLOW_LIB
#define FLOW_LIB

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Adafruit_ADS1015.h>
#include <RotaryEncoder.h>
#include <Encoder.h>
#include <Curves.h>
#include <SysStructs.h>


#define VENTURI_SMALL_DIAM      9e-3f
#define VENTURI_BIG_DIAM        18e-3f
#define AIR_DENSITY             1.2f
#define VENTURI_SMALL_AREA      (VENTURI_SMALL_DIAM*VENTURI_SMALL_DIAM/4)*PI
#define VENTURI_BIG_AREA        (VENTURI_BIG_DIAM*VENTURI_BIG_DIAM/4)*PI
#define AREA_RATIO              4
#define SQ_AREA_RATIO           16
#define MULTIPLIER              5/4096
#define MULTIPLIER_2            0.0078125
#define ORIFICE_PLATE_REF_TEMP  22
#define FLOW_SENSOR_RETRIES     3
#define PERMISSIBLE_VOLUME_ERROR 10

extern double y_0, y_1;

extern Adafruit_ADS1115 ads;

void read_pressure(PressureSensor *, FlowData *);
void read_pressure_2(PressureSensor *, FlowData *);
double calculate_flow_venturi(PressureSensor *);
void calculate_flow_oplate(FlowData *);
int16_t calibrate_pressure_sensor(PressureSensor *);
double arr_average(int16_t *, uint16_t);
double arr_top(double *, uint16_t);
void calculate_flow_state(StepInfo *, SysState *, ODriveArduino *, CurveParams *, FlowData *);
void flow_controller(StepInfo *s, SysState *sys, ControlVals *con, CurveParams *c, CurveParams *n, FlowData *f);
double deg_to_vol(double);
double vol_to_deg(double);
uint8_t calcCRC(char *buff, int num) ;
void calculate_flow_sensirion(FlowData *f, SysState *s);
void sensirion_begin();
void sensirion_restart();


#endif