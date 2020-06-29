#include "PressureSensor.h"
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

double calculate_flow_venturi(PressureSensor *p)
{
  return (VENTURI_BIG_AREA*1000*(sqrt((2 * abs(p->pressure)) / (AIR_DENSITY * ((SQ_AREA_RATIO)-1)))));
}

double calculate_flow_oplate(PressureSensor *p)
{
  if(p->pressure > 0) return (sqrt(abs(p->pressure)/y_0));
  else return (-sqrt(abs(p->pressure)/y_0));
}


void read_pressure(PressureSensor *p)
{
  static int16_t adc[8];
  static int16_t i = 0;
  if(i>7)i = 0;
  adc[i] = analogRead(A0);
  p->pressure_adc = arr_average(adc, 8);
  p->pressure = ((p->pressure_adc - p->openpressure) * MULTIPLIER * 2.222);
  i++;
}

void read_pressure_2(PressureSensor *p)
{
  static int16_t adc[8];
  static int16_t i = 0;
  if(i>7)i = 0;
  adc[i] = abs(ads.readADC_Differential_2_3());
  p->pressure_adc = arr_average(adc, 8);
  if(p->pressure_adc <= (double)(p->openpressure+1)) p->pressure_adc = (double)p->openpressure;
  p->pressure = ((p->pressure_adc - (double)p->openpressure) * MULTIPLIER_2 * 1.25* 10.1972*4.40); // cmH2O
  i++;
}


double arr_average(int16_t arr[], uint16_t size)
{
  double sum = 0;
  double average = 0;
  for(int i =  0; i<size; i++) 
  {
    sum += arr[i];
  }
  average = sum / ((double)size);
  return (average);
}

double arr_top(double arr[], uint16_t size)
{
  double top = 0;
  for(int i =  0; i<size; i++) 
  {
    if(arr[i] > top) top = arr[i];
  }
  return (top);
}

int16_t calibrate_pressure_sensor(PressureSensor *p) 
{
  int count = 0;
  uint32_t timer = 0;
  double open_pressure = 0;
  double average = 0;

  while (true)
  {
    if (timer < millis()) 
    {
      // Serial.println("Entering");
      if (p->id == 0)open_pressure += (double)analogRead(A0);
      else open_pressure += abs((double)ads.readADC_Differential_2_3());
      // Serial.println(open_pressure);
      count++;
      timer = millis() + 50;
    }
    if (count >= 100) 
    {
      p->openpressure = ((int16_t)open_pressure/(count));
      Serial.println(p->openpressure);
      Serial.println(open_pressure, 5);
      break;
    }
  }  
  return ((int16_t)average);
}