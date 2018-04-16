#include "Functions.h"

void transmit(float* data_to_send, uint8_t num_values, HardwareSerial* SerialDevice)
{
    uint16_t num_bytes = num_values*4;
    uint8_t* data_ptr = reinterpret_cast<uint8_t*>(data_to_send);
    
    for (int i = 0;i<num_bytes;i++)
    {
        SerialDevice->write(*(data_ptr + i));
    }
    SerialDevice->write('\n');
}

float get_input(uint8_t pot_pin)
{
    float val;
    
    val = analogRead(pot_pin);

    // This code sets the potentiometer from -1 to 1
    val = (val - 512)/512;

    return val;
}

