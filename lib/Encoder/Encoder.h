#ifndef ENCODER_LIB
#define ENCODER_LIB

#include <RotaryEncoder.h>

//Target deg/s
#define DEG_TO_RAD            PI/180
#define CLICKS_TO_RAD         (2*PI/ENCODER_CPR)
#define RAD_TO_CLICK          (ENCODER_CPR/2*PI)
#define PIN_ENCODER_A         PA7
#define PIN_ENCODER_B         PA6
#define ENCODER_CPR           8000

RotaryEncoder encoder(PIN_ENCODER_B, PIN_ENCODER_A, PB1);