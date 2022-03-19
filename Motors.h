#ifndef MOTORS_H
#define MOTORS_H

/**
* This file is a simple API for controlling the motors
* and motor encoders
* 
* kward
*/

#include "RSLK_Pins.h"
#include "Encoder.h"
#include "Romi_Motor_Power.h"
#include "Bumpers.h"
#include "Ultrasonic.h"

class Motors {

  private:
    const float pulsesPerMotorRev = 3;              //  Pulses per revolution of the motor shaft
    const float pulsesPerWheelRev = 1440;           //  Pulses per revolution of the wheels
    const float wheelDiameter = 6.9999;             //  Wheel diameter in cm
    const float wheelBase = 14;                     //  Wheel base in cm
    const float gearRatio = 120;                    //  120:1 because ratio = (360 / pulses per motor revolution(3)) 
    const int defaultDriveSpeed = 50;               //  Default driving speed
    const int defaultTurnSpeed = 50;                //  Default turning speed
    double cmPerPulse;                              //  Centimeters traveled per pulse
    

  public:
    Romi_Motor_Power leftMotor;                       //  For controlling left wheel
    Romi_Motor_Power rightMotor;                      //  For controlling right wheel
    Bumpers bumpSensors;                              //  Bump sensor interface
    IMU* imu;                                         //  Onboard IMU
    UltrasonicSensor* sonicSensor;                    //  Ultrasonic sensor interface
    double heading;                                   //  Current heading angle
    uint16_t buzzerPin = 2;                           //  GPIO pin for the buzzer

    //  Main constructor
    Motors(IMU* mpu, UltrasonicSensor* sonic) 
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

        // Initialize heading to 0
        heading = 0;

        // Set imu
        imu = mpu;
        sonicSensor = sonic;
    }

    //  Move the robot forward a given number of cm's
    void forward(double dist_cm)
    {      
      unsigned long timestep = 0;             //  Time (in microseconds) since last IMU measurement
      unsigned long prevTimeStep = 0;         //  Time value of previous IMU measurement
      double imuVals[6];                      //  Array for storing IMU metrics
      double startHeading = heading;          //  Initial heading
      
      // Compute number of pulses needed
      int nPulses = floor(dist_cm / cmPerPulse) + defaultDriveSpeed;

      // Reset motor encoder counts
      resetLeftEncoderCnt();
      resetRightEncoderCnt();

      // Set motor direction and enable motors
      leftMotor.directionForward();
      rightMotor.directionForward();
      
      leftMotor.enableMotor();
      rightMotor.enableMotor();

      // Set motor speed (start moving)
      leftMotor.setSpeed(defaultDriveSpeed);
      rightMotor.setSpeed(defaultDriveSpeed);

      Serial.println("Pulses needed to travel " + String(dist_cm) + " cm = " + String(nPulses));

      // Previous Z-axis rotational force
      double prevGz = 0;

     // Wait for motor encoder to read nPulses (distance traveled = dist_cm)
     while(getEncoderLeftCnt() < nPulses && getEncoderRightCnt() < nPulses)
     {
         // Check for bumper collision
         monitorForwardSensors();
//
//        // Poll IMU
//        imu->poll(imuVals);
//        delay(200);
//
//        // Compute timestep
//        prevTimeStep = timestep;
//        timestep = (micros() - prevTimeStep) / 1000000.0;
//          
//        // Update heading
//        heading += (imuVals[5] - prevGz)*timestep;
//
//        // Check heading
//        if(heading > startHeading + 2 || heading < startHeading -  2)
//        {
//          // Check direction of error
//          if(heading < startHeading - 2)
//          {
//            // Off to the left, slow down right motor
//            leftMotor.setSpeed(int(defaultDriveSpeed / 2));
//          }
//          else if(heading > startHeading + 2)
//          {
//            // Off to the left, slow down right motor
//            rightMotor.setSpeed(int(defaultDriveSpeed / 2));
//          }
//        }
//
//        // Store Gz for next measurement
//        prevGz = imuVals[5];
     }
     
      // Stop motors
      leftMotor.disableMotor();
      rightMotor.disableMotor();
      
      Serial.println("Actual pulses measured: L = " + String(getEncoderLeftCnt()) + " R = " + String(getEncoderRightCnt()));
    }

    void reverse(double dist_cm)
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

      // Set motor speed (start moving)
      leftMotor.setSpeed(defaultTurnSpeed-10);
      rightMotor.setSpeed(defaultTurnSpeed-10);


     // Wait for motor encoder to read nPulses (distance traveled = dist_cm)
     while(getEncoderLeftCnt() < nPulses && getEncoderRightCnt() < nPulses)
     {
        delay(1);
     }
     
      // Stop motors
      leftMotor.disableMotor();
      rightMotor.disableMotor();
      
      Serial.println("Pulses needed to travel " + String(dist_cm) + "cm = " + String(nPulses));
      Serial.println("Actual pulses measured: L = " + String(getEncoderLeftCnt()) + " R = " + String(getEncoderRightCnt()));
    }

    // Turn a given number of degrees (negative=left turn, positive=right turn)
    void turn(float nDeg)
    {
      unsigned long timestep = 0;
      unsigned long prevTimeStep = 0;
      double imuVals[6];
      
      // Error check
      if(nDeg < -359)
      {
        nDeg = -359;
      }
      if(nDeg > 359)
      {
        nDeg = 359;
      }

      /* 
      *  Compute number of pulses needed
      *  
      *  nPulses = (n + wheelbase)*pi / wheelDiameter*pi
      */
     int nPulses = int(((abs(nDeg) + wheelBase)*M_PI) / wheelDiameter*M_PI);


      // Reset motor encoder counts
      resetLeftEncoderCnt();
      resetRightEncoderCnt();

      Serial.println("\nNumber of pulses to turn " + String(nDeg) + " degrees = " + String(nPulses));
      
      // Check direction 
      if(nDeg < 0)
      {
        // Set motor directions to turn left
        leftMotor.directionBackward();
        rightMotor.directionForward();

        // Enable motors
        leftMotor.enableMotor();
        rightMotor.enableMotor();

        // Set motor speed (start moving)
        leftMotor.setSpeed(defaultTurnSpeed);
        rightMotor.setSpeed(defaultTurnSpeed);
        
        double prevGz = 0;

        // Wait for motor encoder to read nPulses
        while(getEncoderLeftCnt() < nPulses || getEncoderRightCnt() < nPulses)
        {
          // Check forward facing sensors
          monitorForwardSensors();
          
//          imu->poll(imuVals);
//          delay(100);
//
//          prevTimeStep = timestep;
//          timestep = (micros() - prevTimeStep) / 1000000.0;
//          
//          // Update heading
//          heading += (imuVals[5] - prevGz)*timestep;
          
          if(getEncoderLeftCnt() % int(nPulses/2) == 0)
          {
            leftMotor.setSpeed(int(defaultTurnSpeed/2));
            rightMotor.setSpeed(int(defaultTurnSpeed/2));
          }
          delay(1);
//          prevGz = imuVals[5];
        }

        // Stop motors
        leftMotor.disableMotor();
        rightMotor.disableMotor();
        
        Serial.println("Pulses needed to turn " + String(nDeg) + " degrees = " + String(nPulses));
        Serial.println("Actual pulses measured: L = " + String(getEncoderLeftCnt()) + " R = " + String(getEncoderRightCnt()));
      }
      else if(nDeg > 0)
      {
          
        // Set motor directions to turn right
        leftMotor.directionForward();
        rightMotor.directionBackward();

        // Enable motors
        leftMotor.enableMotor();
        rightMotor.enableMotor();

        // Set motor speed (start moving)
        leftMotor.setSpeed(defaultTurnSpeed);
        rightMotor.setSpeed(defaultTurnSpeed);

        double prevGz = 0;

        // Wait for motor encoder to read nPulses
        while(getEncoderLeftCnt() < nPulses || getEncoderRightCnt() < nPulses)
        {
          // Check forward facing sensors
          monitorForwardSensors();
          
//          imu->poll(imuVals);
//          delay(200);
//          
//          prevTimeStep = timestep;
//          timestep = (micros() - prevTimeStep) / 1000000.0;
//          
//          // Update heading
//          heading += (imuVals[5] - prevGz)*timestep;
          
          if(getEncoderLeftCnt() % (nPulses/2) == 0)
          {
            leftMotor.setSpeed(int(defaultTurnSpeed/2));
            rightMotor.setSpeed(int(defaultTurnSpeed/2));
          }

//          prevGz = imuVals[5];
         
        }

        // Stop motors
        leftMotor.disableMotor();
        rightMotor.disableMotor();
        
        //Serial.println("Pulses needed to turn " + String(nDeg) + " degrees = " + String(nPulses));
        //Serial.println("Actual pulses measured: L = " + String(getEncoderLeftCnt()) + " R = " + String(getEncoderRightCnt()));
      }
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
          if(millis() - start >= 50)
          {
            reverse(5); // Reverse ~5cm
          }
        }
      } else {
        bumpSensors.setStatusLed(1);
      }
    }

    // Monitor forward sensors
    // Check objects in front of marv and attempt to handle various situations
    void monitorForwardSensors()
    {
      // Check PIR Sensor
//      if(digitalRead(pirPin) == 1)
//      {
//        Serial.println("PIR Triggered!");
//        alarm(300, buzzerPin, 500, 1, RED_LED);
//      }

      long dist = sonicSensor->measure();
      
      // Check if object is within forward buffer zone
      if(dist <= sonicSensor->forwardOffsetDist + 5)
      {
        // Check if an ultrasonic measurement event has started
        if(sonicSensor->avoids == 0)
        {
          // Start measurement event
          sonicSensor->eventStart = millis();
        }
        
        // Reverse away from the object
        reverse(5);

        // Increment collisions avoided
        sonicSensor->avoids++;

        // Check number of avoidances (i.e. is an object still following MARV)
        if(sonicSensor->avoids >= sonicSensor->warningLimit)
        {
          // Play warning
        }
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
};



#endif
