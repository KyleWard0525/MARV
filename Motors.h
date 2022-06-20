#ifndef MOTORS_H
#define MOTORS_H

/**
  This file is a simple API for controlling the motors
  and motor encoders

  kward
*/

#include "RSLK_Pins.h"
#include "Encoder.h"
#include "Romi_Motor_Power.h"
#include "SensorController.h"

class Motors {

  private:
    const float pulsesPerMotorRev = 3;              //  Pulses per revolution of the motor shaft
    const float pulsesPerWheelRev = 1440;           //  Pulses per revolution of the wheels
    const float wheelDiameter = 6.9999;             //  Wheel diameter in cm
    const float wheelBase = 14;                     //  Wheel base in cm
    const float gearRatio = 120;                    //  120:1 because ratio = (360 / pulses per motor revolution(3))
    int startSpeed = 25;                            //  Initial speed to start moving forward at
    double cmPerPulse;                              //  Centimeters traveled per pulse
    SensorController* sensors;                      //  API for the sensors
  public:
    Romi_Motor_Power leftMotor;                     //  For controlling left wheel
    Romi_Motor_Power rightMotor;                    //  For controlling right wheel
    uint16_t buzzerPin = 2;                         //  GPIO pin for the buzzer
    int driveSpeed = 70;                            //  Default driving speed
    int turnSpeed = 65;                             //  Default turning speed
    bool monitorFront;                              //  Whether or not to monitor forward sensors
    bool logIMU;                                    //  Whether or not to log IMU data

    //  Main constructor
    Motors(SensorController* _sensors)
    {
      sensors = _sensors;

      // Compute cm traveled per pulse
      cmPerPulse = (wheelDiameter * M_PI) / (gearRatio * pulsesPerMotorRev);

      // Setup motor encoders
      setupEncoder(ENCODER_ELA_PIN, ENCODER_ELB_PIN, ENCODER_ERA_PIN, ENCODER_ERB_PIN);
      resetLeftEncoderCnt();
      resetRightEncoderCnt();

      // Initialize motors
      leftMotor.begin(MOTOR_L_SLP_PIN, MOTOR_L_DIR_PIN, MOTOR_L_PWM_PIN); // Params: sleep(enable) pin, direction pin, pwm pin
      rightMotor.begin(MOTOR_R_SLP_PIN, MOTOR_R_DIR_PIN, MOTOR_R_PWM_PIN); // Params: sleep(enable) pin, direction pin, pwm pin

      monitorFront = false;
      logIMU = false;
    }

    //  Move the robot forward a given number of cm's
    void forward(int dist_cm)
    {
      // Compute number of pulses needed
      int nPulses = floor(dist_cm / cmPerPulse) + driveSpeed;

      // Reset motor encoder counts
      resetLeftEncoderCnt();
      resetRightEncoderCnt();

      // Set motor direction and enable motors
      leftMotor.directionForward();
      rightMotor.directionForward();

      leftMotor.enableMotor();
      rightMotor.enableMotor();

      int motorSpeed = startSpeed;
      leftMotor.setRawSpeed(motorSpeed);
      rightMotor.setRawSpeed(motorSpeed);
      
      // Wait for motor encoder to read nPulses (distance traveled = dist_cm)
      while (getEncoderLeftCnt() < nPulses && getEncoderRightCnt() < nPulses)
      {
        // Check forward sensors
        if(monitorFront)
        {
          sensors->monitorForwardSensors();
        }
        
        // Check if line sensors are in calibration mode
        if(sensors->lineTracker->mode == sensors->lineTracker->MODES::Calibrating)
        {
          // Poll sensors to update calibration
          sensors->lineTracker->pollSensors();
        }

        //  Check if IMU data should be recorded
        if(logIMU)
        {
          // Check if imu should be polled, in accordance with imu session
          if((millis() - sensors->imu->session->getStartTime()) % sensors->imu->session->sampleRateMs == 0)
          {
           // Poll imu and save data in current imu session
           sensors->imu->poll();
          }
        }

        // Gradually increase speed
        if ((motorSpeed < driveSpeed) && (getEncoderLeftCnt() % nPulses == 0 || getEncoderRightCnt() % nPulses == 0))
        {
          motorSpeed++;
          leftMotor.setRawSpeed(motorSpeed+1);
          rightMotor.setRawSpeed(motorSpeed-1);
        }

        // Check if motors are turning at different speeds
        if (getEncoderLeftCnt() > getEncoderRightCnt())
        {
          leftMotor.setRawSpeed(motorSpeed - 1);
          rightMotor.setRawSpeed(motorSpeed);
        }
        else if (getEncoderRightCnt() > getEncoderLeftCnt())
        {
          rightMotor.setRawSpeed(motorSpeed - 4);
          leftMotor.setRawSpeed(motorSpeed);
        }

        
        
        delay(5);
      }

      // Stop motors
      rightMotor.disableMotor();
      leftMotor.disableMotor();

    }

    void reverse(int dist_cm)
    {
      // Compute number of pulses needed
      int nPulses = floor(dist_cm / cmPerPulse);

      // Reset motor encoder counts
      resetLeftEncoderCnt();
      resetRightEncoderCnt();

      // Set motor direction and enable motors
      leftMotor.directionBackward();
      rightMotor.directionBackward();

      leftMotor.enableMotor();
      rightMotor.enableMotor();

      int motorSpeed = startSpeed + 10;
      leftMotor.setRawSpeed(motorSpeed);
      rightMotor.setRawSpeed(motorSpeed);

      // Wait for motor encoder to read nPulses (distance traveled = dist_cm)
      while (getEncoderLeftCnt() < nPulses && getEncoderRightCnt() < nPulses)
      {
        //  Check if IMU data should be recorded
        if(logIMU)
        {
          // Check if imu should be polled, in accordance with imu session
          if((millis() - sensors->imu->session->getStartTime()) % sensors->imu->session->sampleRateMs == 0)
          {
           // Poll imu and save data in current imu session
           sensors->imu->poll();
          }
        }

        // Gradually increase speed
        if ((motorSpeed < driveSpeed) && (getEncoderLeftCnt() % nPulses == 0 || getEncoderRightCnt() % nPulses == 0))
        {
          motorSpeed++;
          leftMotor.setRawSpeed(motorSpeed+1);
          rightMotor.setRawSpeed(motorSpeed-1);
        }
        
        // TODO: Monitor rear sensors
        delay(5);
      }

      // Stop motors
      leftMotor.disableMotor();
      rightMotor.disableMotor();
    }

    // Turn a given number of degrees (negative=left turn, positive=right turn)
    void turn(int nDeg)
    {
      // Error check
      if (nDeg < -359)
      {
        nDeg = -359;
      }
      if (nDeg > 359)
      {
        nDeg = 359;
      }

      /*
         Compute number of pulses needed

         nPulses = (n + wheelbase)*pi / wheelDiameter*pi
      */
      int nPulses = int(((abs(nDeg) + wheelBase) * M_PI) / wheelDiameter * M_PI);


      // Reset motor encoder counts
      resetLeftEncoderCnt();
      resetRightEncoderCnt();
      
      // Check direction
      if (nDeg < 0)
      {
        // Set motor directions to turn left
        leftMotor.directionBackward();
        rightMotor.directionForward();

        // Enable motors
        leftMotor.enableMotor();
        rightMotor.enableMotor();

        // Set motor speed (start moving)
        leftMotor.setRawSpeed(turnSpeed);
        rightMotor.setRawSpeed(turnSpeed);

        // Wait for motor encoder to read nPulses
        while (getEncoderRightCnt() < nPulses && getEncoderRightCnt() < nPulses)
        {
          // Check forward sensors
          if(monitorFront)
          {
            sensors->monitorForwardSensors();
          }
          
           //  Check if IMU data should be recorded
          if(logIMU)
          {
            // Check if imu should be polled, in accordance with imu session
            if((millis() - sensors->imu->session->getStartTime()) % sensors->imu->session->sampleRateMs == 0)
            {
             // Poll imu and save data in current imu session
             sensors->imu->poll();
            }
          }

          delay(1);
        }

        // Stop motors
        leftMotor.disableMotor();
        rightMotor.disableMotor();
      }
      else if (nDeg > 0)
      {

        // Set motor directions to turn right
        leftMotor.directionForward();
        rightMotor.directionBackward();

        // Enable motors
        leftMotor.enableMotor();
        rightMotor.enableMotor();

        // Set motor speed (start moving)
        leftMotor.setRawSpeed(turnSpeed);
        rightMotor.setRawSpeed(turnSpeed);

        // Wait for motor encoder to read nPulses
        while (getEncoderLeftCnt() < nPulses && getEncoderRightCnt() < nPulses)
        {
          // Check forward sensors
          if(monitorFront)
          {
            sensors->monitorForwardSensors();
          }
          
          //  Check if IMU data should be recorded
          if(logIMU)
          {
            // Check if imu should be polled, in accordance with imu session
            if((millis() - sensors->imu->session->getStartTime()) % sensors->imu->session->sampleRateMs == 0)
            {
             // Poll imu and save data in current imu session
             sensors->imu->poll();
            }
          }

          

          delay(1);

        }

        // Stop motors
        leftMotor.disableMotor();
        rightMotor.disableMotor();
      }
    }





    //   Getters and Setters   //
    float getWheelDiameter()
    {
      return wheelDiameter;
    }

    float getWheelBase()
    {
      return wheelBase;
    }

    float getGearRatio()
    {
      return gearRatio;
    }

    float getPulsesPerMotorRev()
    {
      return pulsesPerMotorRev;
    }

    void setStartSpeed(int speed)
    {
      startSpeed = speed;
    }
};



#endif
