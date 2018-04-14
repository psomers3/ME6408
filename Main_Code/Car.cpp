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
    
    float velx = trailer_vel*cos(h_angle) + omega_t * l * sin(h_angle);
    float vely = -trailer_vel*sin(h_angle) + omega_t*l*cos(h_angle) - omega_c*l_two;
    
    if(trailer_vel < 0) return -sqrt(sq(velx) + sq(vely));
    else return sqrt(sq(velx) + sq(vely));
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
    float error = m_speed - get_speed();
    float drive_input = (error * -5 ) + m_integral_speed_error * -10;
     
    drive(drive_input);
    // Clamping anti-windup:
    if((abs(drive_input) > .5) && (error * m_integral_speed_error>0)){}
    else 
    {
      m_integral_speed_error += error*(1./SENSORSAMPLINGFREQ);  // Update integral error
    }
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
    m_speed = percent_speed * MAXSPEED;
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

void Car::assist_controller(float Kp, float Ki, float desired)
{
    if (abs(desired) > MAXHITCHANGLE) desired = (desired/abs(desired))*MAXHITCHANGLE;
     
    float error = desired - m_trailer->get_hitch_angle();
    set_steering((error * Kp ) + m_integral_error * Ki * get_speed());
    // Clamping anti-windup:
    if((abs(m_percent_steer) > 1) && (error * m_integral_error>0)){}
    else m_integral_error += error*(1./SENSORSAMPLINGFREQ);  // Update integral error
}

void Car::straight_line_control(float kp, float ki)
{
    //Ki is used here as a proportional gain for the angle
    float desired_ycoord = m_tan_propogation_angle*(m_trailer->get_xpos() - m_propogation_pointx)
                      + m_propogation_pointy;
    float pos_error = desired_ycoord - m_trailer->get_ypos();
    float angle_error = m_propogation_angle - m_trailer->get_inertial_angle();
    float input = pos_error * m_input->get_kp_pos() + angle_error * m_input->get_kp_yaw()
                  + m_integral_pos_error * m_input->get_ki_pos()*get_speed()
                  + m_integral_yaw_error * m_input->get_ki_yaw()*get_speed();              
    assist_controller(kp,ki,input);
    
    //Anti-windup for integral controls
    if((abs(input) > MAXHITCHANGLE) && (pos_error * m_integral_pos_error>0)){}
    else m_integral_pos_error += pos_error*(1./SENSORSAMPLINGFREQ);  // Update integral error
    if((abs(input) > MAXHITCHANGLE) && (angle_error * m_integral_yaw_error>0)){}
    else m_integral_yaw_error += angle_error*(1./SENSORSAMPLINGFREQ);  // Update integral error
    
}

void Car::set_propogation_point()
{
    m_propogation_pointx = m_trailer->get_xpos();
    m_propogation_pointy = m_trailer->get_ypos();
    m_propogation_angle = m_trailer->get_inertial_angle();
    m_tan_propogation_angle = tan(m_propogation_angle);
}

void Car::zero_integral_error()
{
    m_integral_error = 0;
    m_integral_pos_error = 0;
    m_integral_yaw_error = 0;
    m_integral_speed_error = 0;
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
        set_steering(-m_input->get_steering());
        break;
      case CarController::ZERO:
        if(!m_steering.attached()) m_steering.attach(m_servo_pin);
        set_speed(0);
        set_steering(0);
        break;
      case CarController::ASSIST_1:
        if(!m_steering.attached()) m_steering.attach(m_servo_pin);
        set_speed(m_input->get_velocity());
        assist_controller(m_Kp,m_Ki,desired_hitch_angle()); 
        break;
      case CarController::ASSIST_2:
        if(!m_steering.attached()) m_steering.attach(m_servo_pin);
        set_speed(m_input->get_velocity());
        assist_controller(m_input->get_kp(),m_input->get_ki(),desired_hitch_angle()); 
        break;
      case CarController::STRAIGHT_CONTROL:
        //*** MAY NEED TO ZERO INTEGRAL CONTROL HERE WHEN SWITCHING CONTROLLERS******
        if(!m_steering.attached()) m_steering.attach(m_servo_pin);
        set_speed(m_input->get_velocity());
        if(get_input_radius()>100) straight_line_control(m_input->get_kp(),m_input->get_ki());
        else assist_controller(m_input->get_kp(),m_input->get_ki(),desired_hitch_angle());
        break;
  }
}


