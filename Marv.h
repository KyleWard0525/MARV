#ifndef MARV_H
#define MARV_H

/**
 * Interface for controlling MARV (Mildly Autonoumous Robotic Vehicle)
 * 
 * kward
 */
#include <stdio.h>
#include <string.h>
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

using namespace std;

// Main class for controlling the robot
class Marv {

  private:
    uint16_t buzzerPin;                   //  GPIO pin for buzzer
    uint16_t morseLedPin;                 //  GPIO pin for onboard LED used for communicating in morse code
    const float wheelBase = 14;           //  Wheel base in cm
    Bumpers bumpSensors;                  //  Interface for the bump sensors
    
    
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
    Motors* motors;                       //  API for precise motor controls
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
      feeling = Idle;

      // Initialize LCD screen
      LiquidCrystal_I2C lcdI2C_module(0x27,16,2);
      lcd = new LCD(&lcdI2C_module);
      Serial.println("LCD initialized.");
      
      // Initialize morse object
      morse = new Morse(buzzerPin, morseLedPin);
      Serial.println("Morse initialized.");
      
      // Initialize IMU
      imu = new IMU(&serialBus);
      Serial.println("IMU initialized.");

      //Initialize motors
      motors = new Motors(imu);
      
      // Initialize ultrasonic sensor
      sonicSensor = new UltrasonicSensor(ePin, tPin);
      Serial.println("Ultrasonic sensor initialized.");

      delay(500);
      lcd->showMessage(String("Distance: " + String(sonicSensor->measure()) + " cm").c_str(), 5000);
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
            motors->reverse(5); // Reverse ~5cm
          }
        }
      } else {
        bumpSensors.setStatusLed(1);
      }
    }

    // Check distance to objects in front of marv and attempt to handle various situations
    void monitorForwardSensor()
    {
      // Check distance to nearest object
      
    }

    void displayPitchRoll()
    {
      delay(1000);
      // Get pitch and roll
      double pyr[3];
      imu->getPitchYawRoll(pyr);

      // Write pitch onto screen
      const char* msg = String("Pitch: " + String(pyr[0])).c_str();
      this->lcd->on();
      delay(1000);
      this->lcd->off();
    }
};



#endif      //  End MARV_H
