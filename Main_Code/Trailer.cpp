#include "Trailer.hpp"


uint8_t Trailer::m_num_trailers = 0;
Trailer* Trailer::m_Trailer_ptr[MaxNumofTrailers];


Trailer::Trailer(float track_width,float hitch_to_axle,float wheel_dia, AngleSensor* hitch, AngleSensor* LWheel, AngleSensor* RWheel):
    m_track_width(track_width), m_hitch_to_axle(hitch_to_axle), m_wheel_dia(wheel_dia),
    m_hitch_sensor(hitch), m_LWheel(LWheel), m_RWheel(RWheel)
    {
        m_Trailer_ptr[m_num_trailers] = this;
        m_num_trailers++;
      }

float Trailer::get_hitch_angle()
{
    return -m_hitch_sensor->get_angle();
}

uint8_t Trailer::get_num_trailers()
{
   return m_num_trailers;   
}

Trailer* Trailer::get_trailer_ptr(uint8_t index)
{
  return m_Trailer_ptr[index];
}

float Trailer::get_yaw_rate()
{
   //this has a minus because the wheels are spinning opposite directions to go forward
    return -((m_RWheel->get_velocity() + m_LWheel->get_velocity())*m_wheel_dia)/(2*m_track_width);
}

float Trailer::get_velocity()
{
  //this has a minus because the wheels are spinning opposite directions to go forward
    return ((m_LWheel->get_velocity() - m_RWheel->get_velocity())/2) * (m_wheel_dia/2); 
}

void Trailer::update_trailer(float freq)
{
  for (int i=0; i<Trailer::get_num_trailers(); i++)
  {
    Trailer::get_trailer_ptr(i)->update_angle(freq);
    Trailer::get_trailer_ptr(i)->update_position(freq);
  }
}

void Trailer::update_angle(float freq)
{
    m_inertial_angle += get_yaw_rate() * (1/freq);
}

void Trailer::update_position(float freq)
{
    m_xpos+= get_velocity()*cos(get_inertial_angle())*(1/freq);
    m_ypos+= get_velocity()*sin(get_inertial_angle())*(1/freq);
}

float Trailer::get_inertial_angle()
{
    return m_inertial_angle;  
}

float Trailer::get_hitch_rate()
{
    return m_hitch_sensor->get_velocity();
}

float Trailer::get_hitch_dist()
{
    return m_hitch_to_axle;  
}

float Trailer::get_xpos()
{
    return m_xpos;
}

float Trailer::get_ypos()
{
    return m_ypos;
}

void Trailer::zero()
{
    AngleSensor::zero_all();
    m_inertial_angle = 0;
    m_xpos = 0;
    m_ypos = 0;
}

