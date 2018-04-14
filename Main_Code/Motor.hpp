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
    /// Constructor
    /**
     * Creates an object Motor
     * @param pwm_pin the digital output pin number to be used for pwm speed control.
     * @param dir_pin the digital output pin number to be used to control the direction of spin of the motor.
     */
    Motor(uint8_t pwm_pin, uint8_t dir_pin);

    /// Drives the Motor at specific speed and direction
    /**
     * This function will drive the motor at a percentage of full speed according to duty_cycle. The motor will 
     * reverse directions depending on the sign of duty_cyle
     * @param The duty cycle 
     */
    void set_speed(float duty_cycle);
};
#endif /* Motor_hpp */
