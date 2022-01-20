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
      if(bumpSensors.checkForCollision() == true)
      {
        bumpSensors.alert(buzzerPin);
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

      // Sleep for duration_ms then turn of motors
      delay(duration_ms);
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

    // Turn left for a given number of degrees
    void turnLeft(int speed, int nDeg)
    {
      int mappedDelay = map(nDeg, 0, 360, 0, 100*speed);

      // Set motor directions
      rightMotor.directionForward();
      leftMotor.disableMotor();
      rightMotor.enableMotor();

      // Set motor speed (start moving)
      rightMotor.setSpeed(speed);

      // Delay then turn of motors
      delay(mappedDelay);
      rightMotor.disableMotor();
    }
};



#endif      //  End MARV_H
