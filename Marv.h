#ifndef MARV_H
#define MARV_H

/**
 * Interface for controlling MARV (Mildly Autonoumous Robotic Vehicle)
 * 
 * kward
 */
#include <stdio.h>
#include <string.h>
#include <Wire.h>
#include <Math.h>
#include <Romi_Motor_Power.h>
#include <LiquidCrystal_I2C.h>
#include "Energia.h"
#include "GPIO.h"
#include "RSLK_Pins.h"
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
    uint16_t pirPin;                      //  Pin for PIR Sensor
    uint8_t forwardBufferDist;            //  Forward buffer distance in cm
    const float wheelBase = 14;           //  Wheel base in cm
    Bumpers bumpSensors;                  //  Interface for the bump sensors


    //  Struct for controlling MARV's "emotions" and corresponding behavior
    struct Emotions {
      // Enumerator for MARV's emotional state
      enum EMOTIONAL_STATE {
        Idle,             //  Main state, occurs when marv is not doing anything
        Afraid,           //  Someone too close, marv stuck(long period), other
        Angry,            //  Someone way too close, marv stuck(short period), continuously bumping, other
        Bored,            //  Idle for long period of time
        Curious           //  'Go' mode. When marv wants to travel around and explore
      } feeling;

      // Assign generic emotion type display LEDs 
      uint16_t goodLed = GREEN_LED;     //  For good emotions
      uint16_t neutralLed = BLUE_LED;   //  For neutral emotions
      uint16_t badLed = RED_LED;        //  For bad emotions
      uint16_t otherLed = YELLOW_LED;   //  For various other emotions

      // Display emotion using external peripherals
      void displayEmotion()
      {
        // Check current emotion
        switch(feeling) {

          // Idle, MARV is just vibing
          case Idle:
            // Enable 'Good' emotion LED
            digitalWrite(goodLed, HIGH);
            break;
        }
      }

      // Play nervous beeps (intensity between 1-10)
      void nervous(int intensity)
      {
        // Clamp intensity (error check)
        intensity = clamp(intensity, 1, 10);

        // Check intensity
      }
      
    } emotions;
    
  public:

    I2C serialBus;                        //  Reading and writing data of I2C channels
    Motors* motors;                       //  API for precise motor controls
    IMU* imu;                             //  Measuring acceleration and gyro forces
    Morse* morse;                         //  Communcation with the outside world through Morse code
    UltrasonicSensor* frontSonicSensor;   //  Forward facing sensor for measuring distance using ultrasound
    LCD* lcd;                             //  Interface for LCD1602 screen
    
    //  Main constructor
    Marv(uint16_t buzzPin, uint16_t morsePin, uint16_t tPin, uint16_t ePin, uint16_t irPin, LiquidCrystal_I2C* lcdI2C)
    { 
      // Set peripheral pins
      buzzerPin = buzzPin;
      morseLedPin = morsePin;
      pirPin = irPin;

      // Initialize LCD screen
      lcd = new LCD(lcdI2C);
      lcd->showMessage("Booting...", -1, lcd->screenPadding, 0);   //  Show boot message
      Serial.println("LCD initialized.");
      
      // Initialize morse object
      morse = new Morse(buzzerPin, morseLedPin);
      Serial.println("Morse initialized.");
      
      // Initialize IMU
      imu = new IMU(&serialBus);
      
      // Initialize ultrasonic sensors
      frontSonicSensor = new UltrasonicSensor(ePin, tPin);
      frontSonicSensor->bufferDist = 5;
      
      Serial.println("Ultrasonic sensor initialized.");

      //Initialize motors
      motors = new Motors(imu, frontSonicSensor);

      // Set MARV's initial internal states and variables
      emotions.feeling = emotions.Idle;
      forwardBufferDist = frontSonicSensor->offsetDist + frontSonicSensor->bufferDist; // 5cm buffer in front of robot

      // Signal that MARV is ready to go
      signalReady();
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

    // Signal that MARV is ready
    void signalReady()
    {
      // Play 'ready' beep
      morse->i(); 

      // Show message on lcd screen
      lcd->showMessage("Ready!", 1500, 5, 0);
    }

    // Check objects in front of marv and attempt to handle various situations
    void monitorForwardSensors()
    {
      // Check PIR Sensor
//      if(digitalRead(pirPin) == 1)
//      {
//        Serial.println("PIR Triggered!");
//        alarm(300, buzzerPin, 500, 1, RED_LED);
//      }
      
      // Measure distance to nearest object
      long dist = frontSonicSensor->measure();
      lcd->showMessage("Distance: " + String(dist) + "cm", -1, 0, 0); lcd->showMessage("Distance: " + String(dist) + "cm", -1, 0, 0); 

      // Check if object is within forward buffer zone
      if(dist <= forwardBufferDist)
      {
        // Check if an ultrasonic measurement event has started
        if(frontSonicSensor->avoids == 0)
        {
          // Start measurement event
          frontSonicSensor->eventStart = millis();
        }
        
        // Reverse away from the object
        motors->reverse(5);

        // Increment collisions avoided
        frontSonicSensor->avoids++;

        // Check number of avoidances (i.e. is an object still following MARV)
        if(frontSonicSensor->avoids >= frontSonicSensor->warningLimit)
        {
          // Play warning
        }
      }
    }


   void say(String msg)
   {
    lcd->showMessage("Hi", 1000, 1, 0);
   }

    
};



#endif      //  End MARV_H
