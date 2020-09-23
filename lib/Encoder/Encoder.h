#ifndef ENCODER_LIB
#define ENCODER_LIB

#include <RotaryEncoder.h>
#include <SysStructs.h>

//Target deg/s
#define ENCODER_CPR           (163840)
#define DEG_TO_RAD            (PI/180)
#define CLICKS_TO_RAD         ((2*PI)/163840)
#define RAD_TO_CLICK          (163840/(2*PI))
#define PIN_ENCODER_A         PA7
#define PIN_ENCODER_B         PA6
#define ENCODER_WINDOW        50                    //Minimum time to elapse between odrive encoder readings

// extern RotaryEncoder encoder(PIN_ENCODER_B, PIN_ENCODER_A, PB1)
extern int32_t zero_position;

int32_t odrive_read_encoder(HardwareSerial*, ODriveArduino*, uint8_t);
#endif