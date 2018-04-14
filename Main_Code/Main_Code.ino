#include "AngleSensor.hpp"
#include "UserInput.hpp"
#include "Car.hpp"
#include "Trailer.hpp"
#include "Functions.h"

//***Setup all the hardware*********
AngleSensor HitchSensor(19,23); // Encoder 1
AngleSensor LWheel(18,25);      // Encoder 2
AngleSensor RWheel(2,27);       // Encoder 3
UserInput Driver(&Serial2,57600);
Trailer trailer(  .150, // track width
                  .225, // hitch dist
                  .0587, // wheel dia
                  &HitchSensor,
                  &LWheel,
                  &RWheel);
                  
Car car(6, //steering servo signal pin
        13, //motor pwm pin
        28, //motor direction pin
        &trailer, //trailer object
        &Driver, //userinput object
        0.186, //wheelbase
        0.0578); //rear axle to hitch dist
//**********************************

uint8_t state = 1;
uint8_t last_state = 100;
float output[7];

void setup() 
{
  Serial3.begin(57600);
  car.initialize();
  Driver.start_user_input(); //starts timer4 for sampling user input
}

void loop() 
{
  delay(100); //control update rate of main program
  
  state = Driver.get_mode();
  
  //mode switch if changed
  if (state != last_state) //check if new state desired
  {
      switch (state)
      {
          case 1: //detaches servo
            car.set_controller(CarController::CAR_IDLE);
            last_state = state;
            break;
          case 2: //zero's states and detaches servo
            car.zero_integral_error();
            car.set_controller(CarController::CAR_IDLE);
            car.zero();
            last_state = state;
            break;
          case 3: //direct drive controller
            car.zero_integral_error();
            car.set_controller(CarController::DIRECT_DRIVE);
            last_state = state;
            break;
          case 4: //default gain controller 
            car.zero_integral_error();
            car.set_gains(4,-3);
            car.set_controller(CarController::ASSIST_1);
            last_state = state;
            break;
          case 5: //user input gain controller
            car.zero_integral_error();
            car.set_controller(CarController::ASSIST_2);
            last_state = state;
            break;
          case 6: //position control straight line
            car.zero_integral_error();
            car.set_controller(CarController::STRAIGHT_CONTROL);
            last_state = state;
            break;
          default:
            break;
      }
  }

  //transmit data to Simulink
  car.get_outputs(output);
  transmit(output,7,&Serial3);
} 


