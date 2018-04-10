#include "UserInput.hpp"

uint8_t UserInput::m_num_users = 0;
UserInput* UserInput::m_user_ptrs[MaxNumofUsers];

UserInput::UserInput(HardwareSerial* serial_port,uint16_t baudrate):m_serial_port(serial_port)
{
    m_serial_port->begin(baudrate);
    m_user_ptrs[m_num_users] = this;
    m_num_users++;
}

bool UserInput::user_connect()
{
    uint32_t temp = 0;
    m_serial_port->write('x'); //send a test char to slave
    do
    {   
        
        if(m_serial_port->available())
        {
            if (m_serial_port->read() == 'x')
            {
              m_communication_established = true;
            }
            else
            {
              temp = 1000;
            }
        }
        else 
        {
          m_communication_established = false;
        }
        
        temp++;
        if(temp > 1000) break; //exits loop if failed to connect 20 times
        
    } while (!m_communication_established);
    
    return m_communication_established;
}

int UserInput::update_input()
{
    uint8_t buff[22]; // 5 float values + \n chars
    bool buffer_full = false;
    uint8_t index = 0;
    
    //checks if connection to slave is established
    //and tries to connect if not
    if(!m_communication_established) user_connect();
    
    if (m_communication_established)
    {
        m_serial_port->write('y');
        delay(15);
        if (m_serial_port->read() == 'y')
        {
          while (!buffer_full)
          {   
              if (m_serial_port->available())
              {
                  for (int i = 0;i<22;i++)
                  {
                    buff[i] = m_serial_port->read();
                    index++;
                  }
              }
              if (index == 22) buffer_full = true;
          }
          
          if (buff[0] == '\n' && buff[21] == '\n')
          {
              m_velocity_input = *reinterpret_cast<float*>(&buff[1]);
              m_steering_input = *reinterpret_cast<float*>(&buff[5]);
              m_mode = (uint8_t)*reinterpret_cast<float*>(&buff[9]);
              m_kp = *reinterpret_cast<float*>(&buff[13]);
              m_ki = *reinterpret_cast<float*>(&buff[17]);
              return 0; //successfully updated values
          }
          else
          {
            return 1; // failed to get new values
          }
        }
        else
        {
          return 3;// didn't read echo properly
        }
    }
    else return 2; //device isn't connected
}

void UserInput::start_user_input()
{
  Timer4.initialize((1./USERSAMPLINGFREQ)*1000000);
  Timer4.attachInterrupt(UserInput::input_updating);
}

void UserInput::input_updating()
{
  //this is wrapped because update_input allows for error checking
  //by returning an integer. Nothing is done with it right now though
  for (int i=0; i<UserInput::get_num_users(); i++)
  {
    UserInput::get_user_ptr(i)->update_input();
  }
}

uint8_t UserInput::get_num_users()
{
   return m_num_users;   
}

UserInput* UserInput::get_user_ptr(uint8_t index)
{
  return m_user_ptrs[index];
}

float UserInput::get_steering()
{
    return m_steering_input;
}

float UserInput::get_velocity()
{
    return m_velocity_input;
}

uint8_t UserInput::get_mode()
{
    return m_mode;
}

bool UserInput::is_connected()
{
  return m_communication_established;
}

float UserInput::get_kp()
{
    return m_kp;
}

float UserInput::get_ki()
{
    return m_ki; 
}

