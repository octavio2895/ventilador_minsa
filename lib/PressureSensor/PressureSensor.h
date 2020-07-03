#ifndef PRESSURE_LIB
#define PRESSURE_LIB

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Adafruit_ADS1015.h>
#include <StepData.h>
#include <Encoder.h>
#include <Curves.h>


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

extern double y_0;

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

extern Adafruit_ADS1115 ads;

void read_pressure(PressureSensor *, FlowData *);
void read_pressure_2(PressureSensor *, FlowData *);
double calculate_flow_venturi(PressureSensor *);
void calculate_flow_oplate(FlowData *);
int16_t calibrate_pressure_sensor(PressureSensor *);
double arr_average(int16_t *, uint16_t);
double arr_top(double *, uint16_t);
void calculate_flow_state(StepInfo *, CurveParams *, FlowData *);
void flow_controller(StepInfo *, CurveParams *, CurveParams *, FlowData *);
double deg_to_vol(double);
double vol_to_deg(double);

#endif