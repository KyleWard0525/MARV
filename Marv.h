#ifndef MARV_H
#define MARV_H

/**
 * Interface for controlling MARV (Mildly Autonoumous Robotic Vehicle)
 * 
 * kward
 */
#include <Romi_Motor_Power.h>
#include <Wire.h>
#include <Math.h>
#include "Energia.h"
#include "GPIO.h"
#include "RSLK_Pins.h"
#include "LiquidCrystal_I2C.h"
#include "Utils.h"
#include "Bumpers.h"
#include "I2C.h"
#include "IMU.h"
#include "Morse.h"
#include "Ultrasonic.h"
#include "LCD.h"
#include "Motors.h"

// Main class for controlling the robot
class Marv {

  private:
    uint16_t buzzerPin;                   //  GPIO pin for buzzer
    uint16_t morseLedPin;                 //  GPIO pin for onboard LED used for communicating in morse code

    Bumpers bumpSensors;                  //  Interface for the bump sensors
    Motors motors;                        //  API for precise motor controls
    
    
  public:

    // Enumerator for MARV's emotional state
    enum EMOTIONAL_STATE {
      Idle,             //  Main state, occurs when marv is not doing anything
      Afraid,           //  Someone too close, marv stuck(long period), other
      Angry,            //  Someone way too close, marv stuck(short period), continuously bumping, other
      Bored,            //  Idle for long period of time
      Curious           //  'Go' mode. When marv wants to travel around and explore
    } feeling;

  
    I2C serialBus;                        //  Reading and writing data of I2C channels
    IMU* imu;                             //  Measuring acceleration and gyro forces
    Morse* morse;                         //  Communcation with the outside world through Morse code
    UltrasonicSensor* sonicSensor;        //  For measuring distance using ultrasound
    LCD* lcd;                             //  Interface for LCD1602 screen
    
    //  Main constructor
    Marv(uint16_t buzzPin, uint16_t morsePin, uint16_t tPin, uint16_t ePin)
    { 
      // Set peripheral pins
      buzzerPin = buzzPin;
      morseLedPin = morsePin;
      

      // Set MARV's initial internal states
      this->feeling = Idle;

      // Initialize LCD screen
      LiquidCrystal_I2C lcdI2C_module(0x27,16,2);
      this->lcd = new LCD(&lcdI2C_module);

      Serial.println("LCD initialized.");

      // Initialize morse object
      this->morse = new Morse(buzzerPin, morseLedPin);

      // Initialize IMU
      this->imu = new IMU(&serialBus);
      
      // Initialize ultrasonic sensor
      this->sonicSensor = new UltrasonicSensor(ePin, tPin);

      Serial.println("Ultrasonic sensor initialized.");
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
            reverse(20, 1);
          }
        }
        

        // If no one saves the robot, try to save itself
        //reverse(25, 1);
      } else {
        bumpSensors.setStatusLed(1);
      }
    }

    // Check distance to objects in front of marv and attempt to handle various situations
    void monitorForwardSensor()
    {
      // Check distance to nearest object
      
    }


    // Move forward at a given speed for a given time (in seconds)
    void forward(int speed, float duration)
    {
      // Compute movement duration in ms
      int duration_ms = int(duration*1000); 
      
      // Set motor direction and enable motors
      motors.leftMotor.directionForward();
      motors.rightMotor.directionForward();
      
      motors.leftMotor.enableMotor();
      motors.rightMotor.enableMotor();

      // Set motor speed (start moving)
      motors.leftMotor.setSpeed(speed);
      motors.rightMotor.setSpeed(speed);

      // Loop through duration_ms where i is 1ms
      for(int i = 0; i < duration_ms; i++)
      {
        // Check if robot bumps into something
        checkBumpers();

        // Sleep for 5ms
        delay(1);
      }
      
      motors.leftMotor.disableMotor();
      motors.rightMotor.disableMotor();
    }

    // Move forward at a given speed for a given time (in seconds)
    void reverse(int speed, float duration)
    {
      // Compute movement duration in ms
      int duration_ms = int(duration*1000); 
      
      // Set motor direction and enable motors
      motors.leftMotor.directionBackward();
      motors.rightMotor.directionBackward();
      
      motors.leftMotor.enableMotor();
      motors.rightMotor.enableMotor();

      // Set motor speed (start moving)
      motors.leftMotor.setSpeed(speed);
      motors.rightMotor.setSpeed(speed);

      // Sleep for duration_ms then turn of motors
      delay(duration_ms);
      motors.leftMotor.disableMotor();
      motors.rightMotor.disableMotor();
    }

    // Turn left at a given speed
    void turnLeft(int speed, float duration)
    {
      // Compute movement duration in ms
      int duration_ms = int(duration*1000); 
      
      // Set motor directions
      motors.leftMotor.directionBackward();
      motors.rightMotor.directionForward();
      motors.leftMotor.enableMotor();
      motors.rightMotor.enableMotor();

      // Set motor speed (start moving)
      motors.leftMotor.setSpeed(speed);
      motors.rightMotor.setSpeed(speed);

      // Loop through duration_ms where i is 1ms
      for(int i = 0; i < duration_ms; i++)
      {
        // Check if robot bumps into something
        checkBumpers();

        // Sleep for 5ms
        delay(1);
      }
      
      motors.leftMotor.disableMotor();
      motors.rightMotor.disableMotor();
    }

    // Turn right at a given speed
    void turnRight(int speed, float duration)
    {
      // Compute movement duration in ms
      int duration_ms = int(duration*1000); 
      
      // Set motor directions
      motors.leftMotor.directionForward();
      motors.rightMotor.directionBackward();
      motors.leftMotor.enableMotor();
      motors.rightMotor.enableMotor();

      // Set motor speed (start moving)
      motors.leftMotor.setSpeed(speed);
      motors.rightMotor.setSpeed(speed);

      // Loop through duration_ms where i is 1ms
      for(int i = 0; i < duration_ms; i++)
      {
        // Check if robot bumps into something
        checkBumpers();

        // Sleep for 5ms
        delay(1);
      }
      
      motors.leftMotor.disableMotor();
      motors.rightMotor.disableMotor();
    }
};



#endif      //  End MARV_H
