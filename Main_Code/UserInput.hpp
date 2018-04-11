//
//  UserInput.hpp
//  
//
//  Created by Peter Somers on 3/27/18.
//

#ifndef UserInput_hpp
#define UserInput_hpp

#include <Arduino.h>
#include "TimerFour.h"

#define USERSAMPLINGFREQ 10 //Hz
#define MaxNumofUsers 1

class UserInput
{
private:
    static UserInput* m_user_ptrs[];
    static uint8_t m_num_users;
    float m_velocity_input;
    float m_steering_input;
    bool m_communication_established;
    HardwareSerial* m_serial_port;
    int update_input();
    static uint8_t get_num_users();
    static UserInput* get_user_ptr(uint8_t index);
    uint8_t m_mode;
    float m_kp;
    float m_ki;
    
public:
    UserInput(HardwareSerial* serial_port, uint16_t baudrate);
    //~UserInput();
    bool user_connect();
    float get_steering();
    float get_velocity();
    uint8_t get_mode();
    bool is_connected();
    void start_user_input();
    static void input_updating();
    float get_kp();
    float get_ki();
    
};


#endif /* UserInput_hpp */
