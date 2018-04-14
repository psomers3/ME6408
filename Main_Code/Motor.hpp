//  Created by Peter Somers on 3/29/18.
//
#ifndef Motor_hpp
#define Motor_hpp
#include <Arduino.h>

#define MAXPWM 0.75

class Motor
{
private:
    uint8_t m_pwm_pin;
    uint8_t m_dir_pin;

public:
    //constructor
    Motor(uint8_t pwm_pin, uint8_t dir_pin);
    
    void set_speed(float duty_cycle);
};
#endif /* Motor_hpp */
