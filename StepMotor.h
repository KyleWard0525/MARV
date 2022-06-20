#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

/**
 * An API for interfacing with the 2BYJ-48 Stepper motor
 * 
 * kward
 */

#include <Stepper.h>
#include "Energia.h"
#include "Utils.h"


class StepMotor {
  private: 
    const uint32_t stepsPerRev = 64;
    const uint32_t stepsPerTurn = 2048;
    const uint32_t stepsPerDeg = ceil(stepsPerTurn / 360);
    const double degPerStep = ceil(360.0 / 2048.0);
    const uint32_t minRPM = 50;
    const uint32_t maxRPM = 300;
    uint32_t rpm;
    pins_t pins;
    Stepper* _stepper;
  
  public:

    StepMotor(pins_t periphs)
    {
      pins = periphs;
      
      // Set initial rpm
      rpm = 300;

      // Initialize internal stepper motor
      _stepper = new Stepper(stepsPerRev, pins.stepIn_1, pins.stepIn_3, pins.stepIn_2, pins.stepIn_4);

      // Set RPM of the motor
      _stepper->setSpeed(rpm);

      Serial.println("Stepper motor initialized.");
    }

    void test()
    {
      _stepper->step(stepsPerTurn);
      off();
    }

    // Turn a specified number of degrees
    void turn(int deg)
    {
      int steps = (deg * stepsPerDeg) + (stepsPerRev * sign(deg)) + ((stepsPerTurn / stepsPerRev / 2) * sign(deg));
      _stepper->step(steps);
      off();
    }



    // Turn off all inputs to the motor driver
    void off()
    {
      digitalWrite(pins.stepIn_1, LOW);
      digitalWrite(pins.stepIn_2, LOW);
      digitalWrite(pins.stepIn_3, LOW);
      digitalWrite(pins.stepIn_4, LOW);
    }

    void to_string()
    {
      Serial.println("\n\nStepper Motor:");
      Serial.println("--------------------");
      Serial.println("Steps per motor revolution: " + String(stepsPerRev));
      Serial.println("Steps per for 1 full rotation: " + String(stepsPerTurn));
      Serial.println("Steps per degree: " + String(stepsPerDeg));
      Serial.println("Degrees per step: " + String(degPerStep));
    }


    uint32_t getRPM()
    {
      return rpm;
    }
  
    void setRPM(uint32_t revs)
    {
      if(revs > maxRPM)
      {
        rpm = maxRPM;
        Serial.println("\n\nWARNING in StepMotor.setRPM(): " + String(revs) + " > maxRPM (300)!\nSetting rpm to maxRPM");
      }
      else if(revs < minRPM)
      {
        rpm = minRPM;
        Serial.println("\n\nWARNING in StepMotor.setRPM(): " + String(revs) + " > maxRPM (300)!\nSetting rpm to maxRPM");
      }
      else {
        rpm = revs;
      } 
    }
};













#endif      //  End STEPPER_MOTOR_H
