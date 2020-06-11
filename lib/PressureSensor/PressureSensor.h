#ifndef PRESSURE_LIB
#define PRESSURE_LIB

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Adafruit_ADS1015.h>


#define VENTURI_SMALL_DIAM    9e-3f
#define VENTURI_BIG_DIAM      18e-3f
#define AIR_DENSITY           1.2f
#define VENTURI_SMALL_AREA    (VENTURI_SMALL_DIAM*VENTURI_SMALL_DIAM/4)*PI
#define VENTURI_BIG_AREA      (VENTURI_BIG_DIAM*VENTURI_BIG_DIAM/4)*PI
#define MULTIPLIER            5/4096
#define MULTIPLIER_2          0.0078125


struct PressureSensor
{
  uint8_t id;
  int16_t openpressure;
  double pressure_adc;
  double pressure;
};

extern Adafruit_ADS1115 ads;

void read_pressure(PressureSensor *);
void read_pressure_2(PressureSensor *);
double calculate_flow(PressureSensor *);
int16_t calibrate_pressure_sensor(PressureSensor *);
double arr_average(int16_t *, uint16_t);

#endif