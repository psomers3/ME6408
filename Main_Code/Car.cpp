//  Car.cpp
//  Created by Peter Somers on 3/29/18.
#include "Car.hpp"

uint8_t Car::m_num_cars = 0;
Car* Car::m_car_ptr[MaxNumofCars];

Car::Car(uint8_t steering_pin, uint8_t motor_pwm_pin, uint8_t motor_dir_pin,
        Trailer* trailer, UserInput* input, float whlbase,float axle_hitch_dist):
        m_motor(motor_pwm_pin,motor_dir_pin),m_servo_pin(steering_pin),m_trailer(trailer),
        m_input(input),m_hitch_dist(axle_hitch_dist), m_wheelbase(whlbase)
    {
        m_controller = CarController::CAR_IDLE;
        m_car_ptr[m_num_cars] = this;
        m_num_cars++;
    }

uint8_t Car::get_num_cars()
{
   return m_num_cars;   
}

Car* Car::get_car_ptr(uint8_t index)
{
  return m_car_ptr[index];
}

void Car::drive(float percent_speed)
{
    m_motor.set_speed(percent_speed);
}

void Car::set_steering(float percent_steer)
{
    m_percent_steer = percent_steer;
}

void Car::update_steering()
{
    if (m_percent_steer < -1 && m_percent_steer > 1)
    {
      m_percent_steer = m_percent_steer/abs(m_percent_steer); 
    }
    float angle = m_percent_steer * SERVOLIMIT;
    angle += SERVOZERO;    
    m_steering.write(angle);
}

float Car::get_hitch_angle()
{
  return m_trailer->get_hitch_angle();
}

float Car::get_speed()
{
    float omega_t = m_trailer->get_yaw_rate();
    float omega_c = m_yaw_rate;
    float h_angle = m_trailer -> get_hitch_angle();
    float l = m_trailer -> get_hitch_dist();
    float l_two = m_hitch_dist;
    float trailer_vel = m_trailer->get_velocity();
    
    float velx = m_trailer->get_velocity()*cos(h_angle) + omega_t * l * sin(h_angle);
    float vely = -trailer_vel*sin(h_angle) + omega_t*l*cos(h_angle) - omega_c*l_two;
    return sqrt(sq(velx) + sq(vely));
}

float Car::get_steer_angle()
{
    return MAXSTEERANGLE*m_input->get_steering();
}

void Car::update_car_states(void)
{
    AngleSensor::sensor_updating(SENSORSAMPLINGFREQ);
    Trailer::update_trailer(SENSORSAMPLINGFREQ);
    for (int i=0; i<Car::get_num_cars(); i++)
    {
      Car::get_car_ptr(i)->update_yaw_rate();
      Car::get_car_ptr(i)->update_speed();
      Car::get_car_ptr(i)->update_steering();
      Car::get_car_ptr(i)->update_control();
    }
}

void Car::update_yaw_rate()
{
    m_yaw_rate = m_trailer->get_hitch_rate() - m_trailer->get_yaw_rate();
}

void Car::update_speed()
{
    drive(m_percent_speed);
}

void Car::initialize()
{
  Timer3.initialize((1./SENSORSAMPLINGFREQ)*1000000);
  Timer3.attachInterrupt(Car::update_car_states);
}


void Car::set_controller(CarController controller)
{
  m_controller = controller;
}

void Car::set_speed(float percent_speed)
{
    m_percent_speed = percent_speed;
}

float Car::get_input_radius()
{
  float steering_percentage = m_input -> get_steering();
  float radius = 0;
  
  if (steering_percentage < -STEERSTRAIGHT)
  {
      radius = (((-steering_percentage - MINRADIUS)/(1-STEERSTRAIGHT))*(MAXRADIUS-MINRADIUS)) - MINRADIUS;
  }
  else if (steering_percentage > STEERSTRAIGHT)
  {
      radius = -(((steering_percentage - MINRADIUS)/(1-STEERSTRAIGHT))*(MAXRADIUS-MINRADIUS)) + MINRADIUS;
  }
  else
  {
      radius = 10000;
  }
  return radius;
}

void Car::get_outputs(float* output_array)
{
    output_array[0] = m_trailer->get_xpos();
    output_array[1] = m_trailer->get_ypos();
    output_array[2] = m_trailer->get_inertial_angle();
    output_array[3] = m_trailer->get_hitch_angle();
    output_array[4] = get_speed();
    output_array[5] = get_input_radius();
}

float Car::desired_hitch_angle()
{
    float R = get_input_radius();
    float L_one = m_hitch_dist;
    float L_two = m_trailer->get_hitch_dist();

    return (PI - acos(L_one/sqrt(L_two*L_two + R*R))
              - acos(L_two/sqrt(L_two*L_two + R*R)))
              * (R/abs(R));
}

void Car::assist_controller_1(float Kp, float Ki)
{
    float desired = desired_hitch_angle();
    float error = desired - m_trailer->get_hitch_angle();
    
    set_steering((error * Kp ) + m_integral_error * Ki * get_speed());

    // Clamping anti-windup:  If control is saturated and error is same sign as
    // integrated error, leave integrated error unchanged.
    if((abs(m_percent_speed) > 1) && (error * m_integral_error>0)){}
    else
    {
        m_integral_error += error*(1/SENSORSAMPLINGFREQ);  // Update integral error
    }
}

void Car::zero_integral_error()
{
    m_integral_error = 0;
}

void Car::zero()
{
    m_trailer->zero();
}

void Car::set_gains(float kp, float ki)
{
    m_Kp = kp;
    m_Ki = ki;
}

void Car::update_control()
{
  switch (m_controller)
  {
      case CarController::CAR_IDLE:
        drive(0);
        m_steering.detach();
        break;
      case CarController::DIRECT_DRIVE:
        if(!m_steering.attached()) m_steering.attach(m_servo_pin);
        set_speed(m_input->get_velocity());
        set_steering(m_input->get_steering());
        break;
      case CarController::ZERO:
        if(!m_steering.attached()) m_steering.attach(m_servo_pin);
        set_speed(0);
        set_steering(0);
        break;
      case CarController::ASSIST_1:
        if(!m_steering.attached()) m_steering.attach(m_servo_pin);
        set_speed(m_input->get_velocity());
        assist_controller_1(m_Kp,m_Ki); 
        break;
      case CarController::ASSIST_2:
        if(!m_steering.attached()) m_steering.attach(m_servo_pin);
        set_speed(m_input->get_velocity());
        assist_controller_1(m_input->get_kp(),m_input->get_ki()); 
        break;
  }
}


