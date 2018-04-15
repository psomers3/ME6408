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
    float m_kp_pos;
    float m_ki_pos;
    float m_kp_yaw;
    float m_ki_yaw;
public:
    ///Constructor
    /**
     * Creates a UserInput Object
     * @param serial_port takes a HardwareSerial port (e.g. Serial2) for streaming incoming user inputs for the vehicle
     * @param baudrate the baudrate to run the UART connect at (e.g. 38400)
     */
    UserInput(HardwareSerial* serial_port, uint16_t baudrate);

    ///Connects the UserInput object to the paired device for UART communication
    /**
     * This function sends a test byte across the serial port and checks for an expected response. A boolean value is return
     * whether or not this operation was successful
     * @return returns a boolean value whether or not the other device is paired and responding
     */   
    bool user_connect();

     ///Returns the current recieved steering input value
     /**
     * This function returns the most recently recieved steering input update. This value is between -1 and 1.
     * @return returns a float value between -1 and 1 representing steering input
     */ 
    float get_steering();

     ///Returns the current recieved velocity input command
     /**
     * This function returns the most recently recieved velocity input value. This is a value between -1 and 1.
     * @return returns a float value between -1 and 1 representing velocity input
     */ 
    float get_velocity();

     ///Returns the current recieved driving mode command
     /**
     * This function returns the most recently recieved mode input value. This should be a value corresponding to the options laid
     * out in Main.cpp
     * @return returns a uint8_t value representing the desired state of the machine
     */
    uint8_t get_mode();
    
    ///Returns the state of the connection to user
    /**
    * This function returns a boolean value that is true if the last call of user_connect() was successful
    * @return returns a boolean value of the state of connection to the user
    */
    bool is_connected();

    ///Initializes sampling of the user's input
    /**
    * This function starts a timer interrupt at the sampling frequency defined in UserInput.hpp using timer4.
    * The static function input_updating() is attached to this interrupt.
    */
    void start_user_input();

    ///Updates the states of the user inputs
    /**
     * This function will cycle through each instance of UserInput and retrieve updated values
     */
    static void input_updating();

    ///Returns the user inputed proportional gain for hitch angle control
    /**
    * This function returns the most recently recieved kp input value.
    * @return returns a float value representing the kp gain
    */
    float get_kp();

    ///Returns the user inputed integral gain for hitch angle control
    /**
    * This function returns the most recently recieved ki input value.
    * @return returns a float value representing the ki gain
    */
    float get_ki();

    ///Returns the user inputed proportional gain for position control
    /**
    * This function returns the most recently recieved kp_pos input value.
    * @return returns a float value representing the kp_pos gain
    */
    float get_kp_pos();

    ///Returns the user inputed integral gain for position control
    /**
    * This function returns the most recently recieved ki_pos input value.
    * @return returns a float value representing the ki_pos gain
    */
    float get_ki_pos();

    ///Returns the user inputed proportional gain for trailer yaw control
    /**
    * This function returns the most recently recieved kp_yaw input value.
    * @return returns a float value representing the kp_yaw gain
    */
    float get_kp_yaw();

    ///Returns the user inputed integral gain for trailer yaw control
    /**
    * This function returns the most recently recieved ki_yaw input value.
    * @return returns a float value representing the ki_yaw gain
    */
    float get_ki_yaw();
    
};


#endif /* UserInput_hpp */
