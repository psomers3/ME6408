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
#define SENSORSAMPLINGFREQ 30 //Hz

#define SERVOZERO 65//63.2 //degree position for straight wheels of servo
#define SERVOLIMIT 33 //degrees of turn in each direction of servo
#define MAXSTEERANGLE 33 //actual steering limit of the tires

#define MINRADIUS 2 //minimun trailer radius (meters)
#define MAXRADIUS 5 //maximum trailer radius before assuming straight
#define STEERSTRAIGHT 0.1 //percent steering input corresponding to switch to infinite radius

#define MAXSPEED 0.5 // meters/sec
#define MAXHITCHANGLE 0.261799 // 15 degrees in radians

enum class CarController : uint8_t
{
    CAR_IDLE = 0x00,
    DIRECT_DRIVE = 0x01,
    ZERO = 0x02,
    ASSIST_1 = 0x03,
    ASSIST_2 = 0x04,
    STRAIGHT_CONTROL = 0x05
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
    
    float m_propogation_pointx;
    float m_propogation_pointy;
    float m_propogation_angle;
    float m_tan_propogation_angle;
    
    float m_Kp; //proportional gain
    float m_Ki; //integral gain
    float m_integral_error;
    float m_integral_pos_error;
    float m_integral_yaw_error;
    float m_integral_speed_error;
    float m_last_steering_sample;
    float m_required_hitch_angle;

    /// Drives the Car object motor
    /**
     * This function converts a percentage input to a percentage of the MAXSPEED defined in Car.hpp and sets the 
     * motor member pwm output accordingly to obtain the desired speed
     * @param percent_speed a value between -1 and 1 to indicate the percentage and direction of the max velocity
     * of the car
     */
    void drive(float percent_speed);

    
    void update_yaw_rate();
    void update_speed();
    void assist_controller(float kp,float ki, float desired);
    void straight_line_control(float kp, float ki);
    float desired_hitch_angle();
    void update_control();
    void update_steering();
    
public:
    /// Constructor.
    /**
     * This function creates an object of Car 
     * @param steering_pin digital output pin number for pwm signal to servo for steering.
     * @param motor_pwm_pin pwm output pin number for motor speed control
     * @param motor_dir_pin digital output pin number to control direction of car velocity
     * @param trailer takes a pointer to a Trailer object to attach to the car
     * @param input takes a pointer to a UserInput object that receives inputs from the user
     * @param whlbase this is a float value in meters for the wheelbase of the car
     * @param axle_hitch_dist a float value representing the longitudinal distance from the rear
     * axle of the car to the hitch rotation point
     */
    Car(uint8_t steering_pin, uint8_t motor_pwm_pin, uint8_t motor_dir_pin,
        Trailer* trailer, UserInput* input, float whlbase,float axle_hitch_dist);

    /// Updates the states of all Car objects
    /**
     * *This function cycles through all instances of Car objects and updates their states, sensor readings,
     *  and control inputs.
     */
    static void update_car_states(void);

    /// Returns the desired hitch angle converted from the user input in the UserInput member
    /**
     * This function will take the percentage of steering input obtained from the user and convert it
     * to a percentage of the Min and Max radius defined in the Car.hpp file.  These radii are based 
     * on physical limitations of the vehicle system.
     * @return Returns the hitch angle from the Trailer member
     */
    float get_hitch_angle();

    /// Initializes the updating of the Car by starting an interrupt to run at a specified sampling frequency
    /**
     * This function will start a periodic interrupt using Timer3 at the sampling frequency defined in Car.hpp
     * and attaches the update_car_states function to this interrupt.
     */
    void initialize();

    /// Sets the current controller to be used by the Car
    /**
     * This function updates the set current controller in the Car object that will be used when the control 
     * input is updated
     * @param controller takes a CarController enum class defined in the Car.hpp file
     */
    void set_controller(CarController controller);

    /// Zeros all integral errors used by all controllers
    /**
     * This function zeros the current integral error values used for updating various controllers throughout
     * the Car object
     */
    void zero_integral_error();

    /// Sets the speed of the Car
    /**
     * This function will set the speed of the car to a percentage of the MAXSPEED in m/s defined in Car.hpp
     * @param percent_speed accepts a float value between -1 and 1
     */
    void set_speed(float percent_speed);

    /// Sets the steering input for the car
    /**
     * This function will set the percent of steering of min/max steering of the class
     * @param percent_steer takes a value bewteen -1 and 1
     */
    void set_steering(float percent_steer);

    /// Returns the desired radius for the trailer path to follow
    /**
     * This function converts the user inputted steering from the UserInput member to a desired radius
     * The Min and Max allowed radii are define in Car.hpp
     * @return returns a trailer path radius in meters
     */
    float get_input_radius();

    /// Returns the speed of the car
    /**
     * This function uses the rotational velocitys of the Trailer member to determine the actual speed of the
     * car.
     * @return returns a float value in m/s of the velocity of the car
     */
    float get_speed();

    /// Populates an array with values to be sent over Bluetooth
    /**
     * This function is used to gather multiple values from the Car object and its member objects 
     * to send over bluetooth
     * @param output_array pointer to an array of floats that will be populated with the new values
     * to send over bluetooth
     */
    void get_outputs(float* output_array);

    /// Zeros the states of the car
    /**
     * This function zeros the states of the car including its member functions (i.e. trailer position is reset
     *  to 0,0 for it's position).
     */
    void zero();

    /// Records the current position and orientation of the Trailer member for straight line propogation
    /**
     * This function records the x position, y position, and yaw angle of the Trailer member for use in determining
     * the desired position for future points
     */
    void set_propogation_point();

    /// Sets the proportional and integral gains for hitch angle control
    /**
     * @param kp the proportional gain for hitch angle control
     * @param ki the integral gain for hitch angle control
     */
    void set_gains(float kp, float ki);
};

#endif /* Car_hpp */
