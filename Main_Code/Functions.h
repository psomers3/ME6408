//
//  Functions.h
//  
//
//  Created by Peter Somers on 3/29/18.
//

#ifndef Functions_h
#define Functions_h
#include "Arduino.h"
void transmit(float* data_to_send, uint8_t num_values, HardwareSerial* BTSerial);
float get_input(uint8_t pot_pin);


#endif /* Functions_h */
