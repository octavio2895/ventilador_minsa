#include "Encoder.h"
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

int32_t odrive_read_encoder(HardwareSerial* ser, ODriveArduino* odrive, uint8_t axis) //TODO make this asynchronous
{
    static uint32_t last_call;
    static int32_t last_pos;
    if(last_call + ENCODER_WINDOW < millis())
    {
        char serial_buf[50];
        snprintf(serial_buf, sizeof(serial_buf), "r axis%d.encoder.shadow_count", axis);
        ser->println(serial_buf);
        last_pos = -odrive->readInt();
        last_call = millis();
        return last_pos;
    }
    else return last_pos;
}