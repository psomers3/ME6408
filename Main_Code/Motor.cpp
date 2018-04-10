//  Created by Peter Somers on 3/29/18.
//

#include "Motor.hpp"

Motor::Motor(uint8_t pwm_pin, uint8_t dir_pin):m_pwm_pin(pwm_pin),m_dir_pin(dir_pin)
{
    pinMode(pwm_pin,OUTPUT);
    pinMode(dir_pin,OUTPUT);
    set_speed(0);
}

void Motor::set_speed(float duty_cycle)
{
    float true_duty_cycle;
    float pwm_value;
    
    if (duty_cycle < 0) digitalWrite(m_dir_pin, HIGH);
    else digitalWrite(m_dir_pin,LOW);
    
    true_duty_cycle = abs(duty_cycle) * MAXPWM;

    if (true_duty_cycle > 1) true_duty_cycle = 1;
    
    pwm_value = true_duty_cycle * 255;
    analogWrite(m_pwm_pin, pwm_value);
}
