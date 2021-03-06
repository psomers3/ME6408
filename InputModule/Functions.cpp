#include "Functions.h"

void transmit(float* data_to_send, uint8_t num_values, SoftwareSerial* BTSerial)
{
    uint16_t num_bytes = num_values*4;
    uint8_t* data_ptr = reinterpret_cast<uint8_t*>(data_to_send);
    
    BTSerial->write('\n');
    for (int i = 0;i<num_bytes;i++)
    {
        BTSerial->write(*(data_ptr + i));
    }
    BTSerial->write('\n');
}

float get_velocity_input(uint8_t vel_pin)
{   
    int8_t offset = 0;
    float val = analogRead(vel_pin) + offset;
 
    if(val > 1024) val = 1024;
    if(val< 0) val = 0;
    // This code sets the potentiometer from -1 to 1
    val = (val - 512)/512;
    
    if (val < -0.5) return -0.3;
    else if(val >0.5) return 0.3;
    else  return 0;

    
}

float get_input(uint8_t pot_pin)
{
    float val;
    int8_t offset = 40;
    val = (analogRead(pot_pin)+offset);

    if(val > 1024) val = 1024;
    if(val< 0) val = 0;
    // This code sets the potentiometer from -1 to 1
    val = ((val - 512)/512);
    // This if statement evaluates for cases < -0.1; > 0.1 and infinite cases(straight line)
    return val;
}

void print_menu(char menu_selection, Serial_* serial_port)
{
  serial_port->write(27);   //Print "esc"
  serial_port->print("[2J");
  serial_port->write(27);
  serial_port->print("[H");     // cursor to home command
  
  serial_port->println("*********Vehicle Options***********");
  
  serial_port->print("(a) Idle"); 
  if(menu_selection == 1) serial_port->println(" <---");
  else serial_port->print("\r\n");

  serial_port->print("(b) Idle and Zero"); 
  if(menu_selection == 2) serial_port->println(" <---");
  else serial_port->print("\r\n");
  
  serial_port->print("(c) Direct Drive");
  if(menu_selection == 3) serial_port->println(" <---");
  else serial_port->print("\n\r");

  serial_port->print("(d) Assist Mode (default gains)");
  if(menu_selection == 4) serial_port->println(" <---");
  else serial_port->print("\n\r");

  serial_port->print("(e) Assist Mode (User inputed gains)");
  if(menu_selection == 5) serial_port->println(" <---");
  else serial_port->print("\n\r");

  serial_port->print("(f) Position Controlled Straight Line");
  if(menu_selection == 6) serial_port->println(" <---");
  else serial_port->print("\n\r");

  serial_port->print("(g) Enter new proportional hitch gain");
  serial_port->print("\n\r");
  
  serial_port->print("(h) Enter new integral hitch gain");
  serial_port->print("\n\r");

  serial_port->print("(m) Enter new proportional position gain");
  serial_port->print("\n\r");

  serial_port->print("(n) Enter new integral position gain");
  serial_port->print("\n\r");

  serial_port->print("(o) Enter new proportional yaw gain");
  serial_port->print("\n\r");

  serial_port->print("(p) Enter new integral yaw gain");
  serial_port->print("\n\r");
  
  serial_port->print("(q) Print current gain values");
  serial_port->print("\n\r");
  serial_port->print("\nSelect Option From Above");
}

