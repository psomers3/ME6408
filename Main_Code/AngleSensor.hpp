#ifndef AngleSensor_hpp
#define AngleSensor_hpp

#define ENCODER_USE_INTERRUPTS
#include "Encoder.h"
#include <Arduino.h>

#define RADIANS
//#define DEGREES

#define MaxNumofSensors 5


class AngleSensor
{
private:
    static AngleSensor* m_AngleSensor_ptr[];
    static uint8_t m_num_sensors;
    float m_velocity; // angular velocity
    float m_last_angle;
    Encoder m_encoder; //encoder object used to get position
    void update_velocity(float sampling_freq);
    static uint8_t get_num_sensors();
    static AngleSensor* get_sensor_ptr(uint8_t index);

    
public:
    //constructor
    AngleSensor(uint8_t pinA, uint8_t pinB);
    //destructor
    ~AngleSensor();
    
    static void sensor_updating(float freq);
    static void zero_all();       //zeros all angle sensors
    static void start_sensors();  //starts updating all sensors
    int32_t get_position();       //encoder read wrapper
    float get_velocity();         //returns angular velocity of sensor
    float get_angle();            //returns sensor angle
    void zero();                  //zeros sensor
    void set_angle(float angle);  //sets current sensor angle
    
};//AngleSensor

#endif /* AngleSensor_hpp */
