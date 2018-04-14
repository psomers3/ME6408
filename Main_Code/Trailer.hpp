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
     */
    Trailer(float track_width,float hitch_to_axle,float wheel_dia, AngleSensor* hitch, AngleSensor* LWheel, AngleSensor* RWheel);

    static void update_trailer(float freq);
    float get_hitch_angle();
    float get_yaw_rate();
    float get_velocity();
    float get_inertial_angle();
    float get_hitch_rate();
    float get_hitch_dist();
    float get_xpos();
    float get_ypos();
    void zero();
};
#endif /* Trailer_hpp */
