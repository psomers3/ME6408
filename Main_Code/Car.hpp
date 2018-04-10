//  Created by Peter Somers on 3/29/18.
//
#ifndef Car_hpp
#define Car_hpp

#include <Servo.h>
#include "TimerThree.h"

#include "Motor.hpp"
#include "Trailer.hpp"
#include "UserInput.hpp"

#define MaxNumofCars 1
#define SENSORSAMPLINGFREQ 15 //Hz

#define SERVOZERO 63.2 //degree position for straight wheels of servo
#define SERVOLIMIT 30 //degrees of turn in each direction of servo
#define MAXSTEERANGLE 33 //actual steering limit of the tires

#define MINRADIUS 1.0 //minimun trailer radius (meters)
#define MAXRADIUS 5.0 //maximum trailer radius before assuming straight
#define STEERSTRAIGHT 0.1 //percent steering input corresponding to switch to infinite radius

enum class CarController : uint8_t
{
    CAR_IDLE = 0x00,
    DIRECT_DRIVE = 0x01,
    ZERO = 0x02,
    ASSIST_1 = 0X03,
    ASSIST_2 = 0X04
};


class Car
{
private:
    Servo m_steering;
    uint8_t m_servo_pin;
    float m_hitch_dist; //0.0578
    float m_wheelbase; //0.186
    static Car* m_car_ptr[];
    static uint8_t m_num_cars;
    static uint8_t get_num_cars();
    static Car* get_car_ptr(uint8_t index);
    Motor m_motor;
    Trailer* m_trailer;
    UserInput* m_input;
    CarController m_controller;
    float m_yaw;
    float m_yaw_rate;
    float m_speed;
    float m_percent_steer;
    float m_percent_speed;

    float m_Kp; //proportional gain
    float m_Ki; //integral gain    
    float m_integral_error;  
    void drive(float percent_speed);
    
    void update_yaw_rate();
    void update_speed();
    void assist_controller_1(float kp,float ki);
    float desired_hitch_angle();
    void update_control();
    void update_steering();
    
public:
    Car(uint8_t steering_pin, uint8_t motor_pwm_pin, uint8_t motor_dir_pin,
        Trailer* trailer, UserInput* input, float whlbase,float axle_hitch_dist);

    void start();
    static void update_car_states(void);
    float get_hitch_angle();
    void initialize();
    void set_controller(CarController controller);
    void zero_integral_error();
    void set_speed(float percent_speed);
    void set_steering(float percent_steer);
    float get_input_radius();
    float get_speed();
    float get_steer_angle(); //this is used for the controller input calculation only
    void get_outputs(float* output_array);
    void zero();
    void set_gains(float kp, float ki);
};

#endif /* Car_hpp */
