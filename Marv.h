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
#include "Servo.h"
#include "GPIO.h"
#include "RSLK_Pins.h"
#include "Utils.h"
#include "SensorController.h"
#include "Morse.h"
#include "LCD.h"
#include "Motors.h"
#include "List.h"
#include "StepMotor.h"

using namespace std;
using namespace arrays;

// Main class for controlling the robot
class Marv {

  private:
    const float wheelBase = 14;           //  Wheel base in cm


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
    pins_t periphs;                       //  Peripheral pins
    I2C serialBus;                        //  Reading and writing data of I2C channels
    Motors* motors;                       //  API for precise motor controls
    Morse* morse;                         //  Communcation with the outside world through Morse code
    LCD* lcd;                             //  Interface for LCD1602 screen
    Servo servo;
    SensorController* sensors;            //  Controller for interfacing with sensor
    StepMotor* stepper;                   //  Stepper motor interface
    
    //  Main constructor
    Marv(pins_t peripherals, LiquidCrystal_I2C* lcdI2C)
    { 
      // Set peripheral pins
      periphs = peripherals;
      servo.attach(periphs.servoPin);
      
      // Initialize sensors
      sensors = new SensorController(periphs, servo);

      // Initialize LCD screen
      lcd = new LCD(lcdI2C);
      Serial.println("LCD initialized.");
      
      // Initialize morse object
      morse = new Morse(periphs.buzzer, periphs.morseLed);
      Serial.println("Morse initialized.");

      // Initialize motors
      motors = new Motors(sensors);
      Serial.println("Motors initialized.");

      // Initialize stepper motor
      stepper = new StepMotor(periphs);

      // Set MARV's initial internal states and variables
      emotions.feeling = emotions.Idle;

      // Signal that MARV is ready to go
      signalReady();
    }
    
    // Signal that MARV is ready
    void signalReady()
    {
      // Show message on lcd screen
      lcd->resetScreen();
      lcd->showMessage("Ready!", 1000, 5, 0);

      // Play 'ready' beep
      morse->i(); 
    }

    
  

};



#endif      //  End MARV_H
