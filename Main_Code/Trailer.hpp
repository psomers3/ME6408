#ifndef Trailer_hpp
#define Trailer_hpp

#include "AngleSensor.hpp"

#define MaxNumofTrailers 1

class Trailer
{
private:
    static Trailer* m_Trailer_ptr[];
    static uint8_t m_num_trailers;
    
    AngleSensor* m_hitch_sensor;
    AngleSensor* m_LWheel;
    AngleSensor* m_RWheel;
    float m_track_width;
    float m_hitch_to_axle;
    float m_wheel_dia;
    float m_inertial_angle;
    void update_angle(float freq);
    void update_position(float freq);
    static uint8_t get_num_trailers();
    static Trailer* get_trailer_ptr(uint8_t index);
    float m_hitch_rate;
    float m_xpos;
    float m_ypos;
    
public:
    /// Constructor
    /**
     * Creates a Trailer object
     * @param track_width The track width from center of wheel to center of wheel in meters
     * @param hitch_to_axle The longitudnal distance from the trailer axle to the center of the hitch in meters
     * @param wheel_dia outer diameter of the trailer wheels in meters
     * @param hitch pointer to an AngleSensor object that is used as the hitch angle sensor
     * @param LWheel pointer to an AngleSensor object that is used as the left wheel speed sensor
     * @param RWheel pointer to an AngleSensor object that is used as the right wheel speed sensor
     */
    Trailer(float track_width,float hitch_to_axle,float wheel_dia, AngleSensor* hitch, AngleSensor* LWheel, AngleSensor* RWheel);

    /// Updates all instantiated trailer objects
    /**
     * This function will update position, velocity, and yaw values of each currently instantiated trailer object
     * @param freq float value for the sampling frequency used for integrating the position of the trailer
     */
    static void update_trailer(float freq);

    /// Returns the hitch AngleSensor angle value
    /**
     * This function returns the angular position of the hitch sensor in Radians or Degrees depending on which is defined in AngleSensor.hpp
     * @return The current hitch angle of the trailer
     */
    float get_hitch_angle();

    /// Returns the yaw rate (z-axis rotation) of the trailer
    /**
     * This function returns the angular rate of change of the trailer object in the inertial frame
     * @return The current yaw rate of the trailer
     */   
    float get_yaw_rate();

    /// Returns the current velocity of the Trailer object
    /**
     * This function returns the velocity of the Trailer in m/s
     * @return The current velocity of the trailer
     */
    float get_velocity();

    /// Returns the current yaw angle of the Trailer
    /** 
     * This function returns the value of the rotation of the trailer about the z-axis in the inertial frame
     * Units depend upon what is defined in AngleSensor.hpp
     * @return The current yaw angle of the trailer
     */
    float get_inertial_angle();

    /// Returns the current angular rate of change of hitch sensor
    /** 
     * This function returns the value of the rotation rate of the hitch AngleSensor
     * Units depend upon what is defined in AngleSensor.hpp
     * @return The current rotation rate of the hitch sensor
     */
    float get_hitch_rate();

    /// Returns the distance from the axle to hitch
    /** 
     * This function returns the longitudnal distance in meters from the trailer axle to hitch.
     * @return The distance from the axle to the hitch rotation point
     */    
    float get_hitch_dist();

    /// Returns the x position of the trailer
    /** 
     * This function returns the position of the center of the trailer axis relative to the inertail frame in the x direction
     * @return the x position of the trailer in meters
     */    
    float get_xpos();
    
    /// Returns the y position of the trailer
    /** 
     * This function returns the position of the center of the trailer axis relative to the inertail frame in the y direction
     * @return the y position of the trailer in meters
     */  
    float get_ypos();
    
    /// Zeros the Trailer state values
    /** 
     * This function zeros relative state values including position, velocity, AngleSensor counts
     */ 
    void zero();
};
#endif /* Trailer_hpp */
