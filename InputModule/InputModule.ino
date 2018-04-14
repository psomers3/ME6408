#include "Functions.h"
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(8, 9); // RX | TX
#define POTPIN 5  //potentiometer input pin
#define VELPIN 4
char received;
char input = 'a';
char state = 1; //state of vehicle
bool entering_new_kp = false;
bool entering_new_ki = false;
bool entering_new_kp_pos = false;
bool entering_new_ki_pos = false;
bool entering_new_kp_yaw = false;
bool entering_new_ki_yaw = false;

bool refresh_menu = true;
//data[velocity,steering,state,kp,ki,kp_pos,kp_yaw,ki_pos,ki_yaw]
float data[]= {0,0,0,5,-3,-3,0,-1.5,0};
String new_gain = "";

uint8_t gain_index = 0;

void setup() 
{
  Serial.begin(57600);
  BTSerial.begin(57600);
  delay(3000); //wait for serial communication to get going
  print_menu(input,&Serial);
}
 
void loop()
{
  data[1] = get_input(POTPIN);
  
  if (BTSerial.available())
  {
      received = BTSerial.read();
      if (received == 'x') //connection check
      {
        BTSerial.write(received);
      }
      else //request for data
      {
        BTSerial.write(received);
        transmit(data,9,&BTSerial);
      }
  }

  if (Serial.available())
  {
      input = Serial.read();
      switch (input)
      {
          case 'i':
              if(data[0] < 1) data[0] += 0.1;
              refresh_menu = true;
              break;
          case 'k':
              if(data[0] > -1) data[0] -= 0.1;
              refresh_menu = true;
              break;
          case 'a': //idle
              state = 1;
              data[0] = 0; //velocity
              refresh_menu = true;
              break;
          case 'b': //idle and zero
              data[0] = 0; //velocity
              refresh_menu = true;
              state = 2;
              break;
          case 'c': //Direct Drive
              state = 3;
              refresh_menu = true;
              break;
          case 'd': //Default gain controller
              state = 4;
              refresh_menu = true;
              break;
          case 'e': //User input controller
              state = 5;
              refresh_menu = true;
              break;
          case 'f': //User input controller
              state = 6;
              refresh_menu = true;
              break;
          case 'g':
              entering_new_kp = true;
              Serial.print("\r\n\nEnter new kp hitch gain: ");
              refresh_menu = false;
              break;
          case 'h':
              entering_new_ki = true;
              Serial.print("\r\n\nEnter new ki hitch gain: ");
              refresh_menu = false;
              break;
          case 'm':
              entering_new_kp_pos = true;
              Serial.print("\r\n\nEnter new kp position gain: ");
              refresh_menu = false;
              break;
          case 'n':
              entering_new_ki_pos = true;
              Serial.print("\r\n\nEnter new ki position gain: ");
              refresh_menu = false;
              break;
          case 'o':
              entering_new_kp_yaw = true;
              Serial.print("\r\n\nEnter new kp yaw gain: ");
              refresh_menu = false;
              break;
          case 'p':
              entering_new_ki_yaw = true;
              Serial.print("\r\n\nEnter new ki yaw gain: ");
              refresh_menu = false;
              break;
          case 'q':
              Serial.println("\r\n\nCurrent Gain Values:");
              Serial.print("kp hitch = ");
              Serial.println(data[3]);
              Serial.print("ki hitch = ");
              Serial.println(data[4]);
              Serial.print("kp position = ");
              Serial.println(data[5]);
              Serial.print("ki position = ");
              Serial.println(data[6]);
              Serial.print("kp yaw = ");
              Serial.println(data[7]);
              Serial.print("ki yaw = ");
              Serial.println(data[8]);
              refresh_menu = false;
              break;                     
          default:
              if(entering_new_kp)
              {
                      if(input == 0x0D) //return char
                      {
                          data[3] = new_gain.toFloat();
                          entering_new_kp = false;
                          refresh_menu = true;
                          new_gain = "";
                      }
                      else
                      {
                          new_gain += input;
                          Serial.write(input);
                      }
               }
               else if(entering_new_ki)
               {
                      if(input == 0x0D) //return char
                      {
                          data[4] = new_gain.toFloat();
                          entering_new_ki = false;
                          refresh_menu = true;
                          new_gain = "";
                      }
                      else
                      {
                          new_gain += input;
                          Serial.write(input);
                      }
               }
               else if(entering_new_kp_pos)
               {
                      if(input == 0x0D) //return char
                      {
                          data[5] = new_gain.toFloat();
                          entering_new_kp_pos = false;
                          refresh_menu = true;
                          new_gain = "";
                      }
                      else
                      {
                          new_gain += input;
                          Serial.write(input);
                      }
               }
               else if(entering_new_ki_pos)
               {
                      if(input == 0x0D) //return char
                      {
                          data[6] = new_gain.toFloat();
                          entering_new_ki_pos = false;
                          refresh_menu = true;
                          new_gain = "";
                      }
                      else
                      {
                          new_gain += input;
                          Serial.write(input);
                      }
               }
               else if(entering_new_kp_yaw)
               {
                      if(input == 0x0D) //return char
                      {
                          data[7] = new_gain.toFloat();
                          entering_new_kp_yaw = false;
                          refresh_menu = true;
                          new_gain = "";
                      }
                      else
                      {
                          new_gain += input;
                          Serial.write(input);
                      }
               }
               else if(entering_new_ki_yaw)
               {
                      if(input == 0x0D) //return char
                      {
                          data[8] = new_gain.toFloat();
                          entering_new_ki_yaw = false;
                          refresh_menu = true;
                          new_gain = "";
                      }
                      else
                      {
                          new_gain += input;
                          Serial.write(input);
                      }
               }
               break;
        }
        
        if(refresh_menu) print_menu(state,&Serial);
        data[2] = state;
  }
}
