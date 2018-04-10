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
bool refresh_menu = true;
//data[velocity,steering,state,kp,ki]
float data[]= {0,0,0,0,0.25};
String kp_gain = "";
String ki_gain = "";
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
        transmit(data,5,&BTSerial);
      }
  }

  if (Serial.available())
  {
      input = Serial.read();
      switch (input)
      {
          case 'i':
              if(data[0] < 1) data[0] += 1;
              break;
          case 'k':
              if(data[0] > 0) data[0] -= 1;
              break;
          case 'a': //idle
              state = 1;
              data[0] = 0; //velocity
              break;
          case 'b': //idle and zero
              data[0] = 0; //velocity
              state = 2;
              break;
          case 'c': //Direct Drive
              state = 3;
              break;
          case 'd': //Default gain controller
              state = 4;
              break;
          case 'e': //User input controller
              state = 5;
              break;
          case 'g':
              entering_new_kp = true;
              Serial.print("\r\n\nEnter new kp gain: ");
              refresh_menu = false;
              break;
          case 'f':
              entering_new_ki = true;
              Serial.print("\r\n\nEnter new ki gain: ");
              refresh_menu = false;
              break;              
          default:
              if(entering_new_kp)
              {
                      if(input == 0x0D) //return char
                      {
                          data[3] = kp_gain.toFloat();
                          entering_new_kp = false;
                          refresh_menu = true;
                          kp_gain = "";
                      }
                      else
                      {
                          kp_gain += input;
                          Serial.write(input);
                      }
               }
               else if(entering_new_ki)
               {
                      if(input == 0x0D) //return char
                      {
                          data[4] = ki_gain.toFloat();
                          entering_new_ki = false;
                          refresh_menu = true;
                          ki_gain = "";
                      }
                      else
                      {
                          ki_gain += input;
                          Serial.write(input);
                      }
               }
               break;
        }
        
        if(refresh_menu) print_menu(state,&Serial);
        data[2] = state;
  }
}
