#ifndef MARV_H
#define MARV_H

/**
 * Interface for controlling MARV (Mildly Autonoumous Robotic Vehicle)
 * 
 * kward
 */
#include <Romi_Motor_Power.h>
#include "Energia.h"
#include "GPIO.h"
#include "RSLK_Pins.h"
#include "Utils.h"
#include "Bumpers.h"


// Main class for controlling the robot
class Marv {

  private:
    int buzzerPin;
    Bumpers bumpSensors;
    Romi_Motor_Power leftMotor;
    Romi_Motor_Power rightMotor;
    
  public:

    //  Main constructor
    Marv(int buzzPin)
    { 
      // Set pins
      buzzerPin = buzzPin;

      // Initialize motors
      leftMotor.begin(MOTOR_L_SLP_PIN,MOTOR_L_DIR_PIN,MOTOR_L_PWM_PIN);  // Params: sleep(enable) pin, direction pin, pwm pin
      rightMotor.begin(MOTOR_R_SLP_PIN,MOTOR_R_DIR_PIN,MOTOR_R_PWM_PIN); // Params: sleep(enable) pin, direction pin, pwm pin
    }



    // Check bumpers for collision and handle response
    void checkBumpers()
    {
      if(bumpSensors.checkForCollision())
      {
        unsigned long start = millis();
        
        while(bumpSensors.checkForCollision())
        {
          //  Play alert
          bumpSensors.alert(buzzerPin);

          // Check if anyone has helped marv in the alloted time
          if(millis() - start >= 250)
          {
            reverse(10, 1);
          }
        }
        

        // If no one saves the robot, try to save itself
        //reverse(25, 1);
      } else {
        bumpSensors.setStatusLed(1);
      }
    }


    // Move forward at a given speed for a given time (in seconds)
    void forward(int speed, float duration)
    {
      // Compute movement duration in ms
      int duration_ms = int(duration*1000); 
      
      // Set motor direction and enable motors
      leftMotor.directionForward();
      rightMotor.directionForward();
      
      leftMotor.enableMotor();
      rightMotor.enableMotor();

      // Set motor speed (start moving)
      leftMotor.setSpeed(speed);
      rightMotor.setSpeed(speed);

      // Loop through duration_ms where i is 1ms
      for(int i = 0; i < duration_ms; i++)
      {
        // Check if robot bumps into something
        checkBumpers();

        // Sleep for 5ms
        delay(1);
      }
      leftMotor.disableMotor();
      rightMotor.disableMotor();
    }

    // Move forward at a given speed for a given time (in seconds)
    void reverse(int speed, float duration)
    {
      // Compute movement duration in ms
      int duration_ms = int(duration*1000); 
      
      // Set motor direction and enable motors
      leftMotor.directionBackward();
      rightMotor.directionBackward();
      
      leftMotor.enableMotor();
      rightMotor.enableMotor();

      // Set motor speed (start moving)
      leftMotor.setSpeed(speed);
      rightMotor.setSpeed(speed);

      // Sleep for duration_ms then turn of motors
      delay(duration_ms);
      leftMotor.disableMotor();
      rightMotor.disableMotor();
    }

    // Turn left at a given speed
    void turnLeft(int speed, float duration)
    {
      // Compute movement duration in ms
      int duration_ms = int(duration*1000); 
      
      // Set motor directions
      leftMotor.directionBackward();
      rightMotor.directionForward();
      leftMotor.enableMotor();
      rightMotor.enableMotor();

      // Set motor speed (start moving)
      leftMotor.setSpeed(speed);
      rightMotor.setSpeed(speed);

      // Delay then turn of motors
      delay(duration_ms);
      leftMotor.disableMotor();
      rightMotor.disableMotor();
    }

    // Turn right at a given speed
    void turnRight(int speed, float duration)
    {
      // Compute movement duration in ms
      int duration_ms = int(duration*1000); 
      
      // Set motor directions
      leftMotor.directionForward();
      rightMotor.directionBackward();
      leftMotor.enableMotor();
      rightMotor.enableMotor();

      // Set motor speed (start moving)
      leftMotor.setSpeed(speed);
      rightMotor.setSpeed(speed);

      // Delay then turn of motors
      delay(duration_ms);
      leftMotor.disableMotor();
      rightMotor.disableMotor();
    }
};



#endif      //  End MARV_H
