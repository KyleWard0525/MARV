#ifndef MOTORS_H
#define MOTORS_H

/**
* This file is a simple API for controlling the motors
* and motor encoders
* 
* kward
*/
#include <Math.h>
#include "RSLK_Pins.h"
#include "Encoder.h"
#include "Romi_Motor_Power.h"

class Motors {

  private:
    const uint16_t pulsesPerMotorRev = 3;             //  Pulses per revolution of the motor shaft
    const uint16_t pulsesPerWheelRev = 1440;          //  Pulses per revolution of the wheels
    const uint8_t wheelDiameter = 7;                  //  Wheel diameter in cm
    const uint16_t gearRatio = 120;                   //  120:1 because ratio = (360 / pulses per motor revolution(3)) 
    double cmPerPulse;                                //  Centimeters traveled per pulse

    

  public:
    Romi_Motor_Power leftMotor;                       //  For controlling left wheel
    Romi_Motor_Power rightMotor;                      //  For controlling right wheel

    Motors() 
    {
        // Compute cm traveled per pulse
        cmPerPulse = (wheelDiameter * M_PI) / (gearRatio * pulsesPerMotorRev);
      
        // Setup motor encoders
        setupEncoder(ENCODER_ELA_PIN,ENCODER_ELB_PIN,ENCODER_ERA_PIN,ENCODER_ERB_PIN);
        resetLeftEncoderCnt();
        resetRightEncoderCnt();

        // Initialize motors
        leftMotor.begin(MOTOR_L_SLP_PIN,MOTOR_L_DIR_PIN,MOTOR_L_PWM_PIN);  // Params: sleep(enable) pin, direction pin, pwm pin
        rightMotor.begin(MOTOR_R_SLP_PIN,MOTOR_R_DIR_PIN,MOTOR_R_PWM_PIN); // Params: sleep(enable) pin, direction pin, pwm pin
    }

    
    void forward(double dist_cm, uint16_t speed)
    {
      // Compute number of pulses needed
      int nPulses = floor(dist_cm / cmPerPulse);

      // Reset motor encoder counts
      resetLeftEncoderCnt();
      resetRightEncoderCnt();

      // Set motor direction and enable motors
      leftMotor.directionForward();
      rightMotor.directionForward();
      
      leftMotor.enableMotor();
      rightMotor.enableMotor();

      // Set motor speed (start moving)
      leftMotor.setSpeed(speed);
      rightMotor.setSpeed(speed);


     // Wait for motor encoder to read nPulses (distance traveled = dist_cm)
     while(getEncoderLeftCnt() < nPulses && getEncoderRightCnt() < nPulses)
     {
        // Do nothing
        ;
     }
     
      // Stop motors
      leftMotor.disableMotor();
      rightMotor.disableMotor();
      
      Serial.println("Pulses needed to travel " + String(dist_cm) + "cm = " + String(nPulses));
      Serial.println("Actual pulses measured: L = " + String(getEncoderLeftCnt()) + " R = " + String(getEncoderRightCnt()));
    }
};














#endif
