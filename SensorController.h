#ifndef SENSOR_CONTROLLER_H
#define SENSOR_CONTROLLER_H
/**
 * Controller for most if not all of MARV's sensors
 * 
 * kward
 */

#include "I2C.h"
#include "IMU.h"
#include "Bumpers.h"
#include "Ultrasonic.h"
#include "Telemetry.h"
#include "Utils.h"
#include "Servo.h"
#include "LineTracker.h"

class SensorController {

  private:
    pins_t sensor_pins;     //  GPIO pins for the sensors

  public:
    I2C serialBus;                          //  I2C API
    IMU* imu;                               //  IMU interface
    UltrasonicSensor* frontSonicSensor;     //  Forward facing sensor for measuring distance using ultrasound
    UltrasonicSensor* rearSonicSensor;      //  Rear facing sensor for measuring distance using ultrasound
    LineTracker*  lineTracker;              //  API for line tracking module
    Bumpers bumpers;                        //  Forward facing bump sensors
    Servo servo;                            //  Servo API for front sonic sensor assembly
    bool monitorBlackLine;                  //  Flag for if robot should activate line following if found


    // Initiate callback variables for Motor functions since Energia doesn't allow inheritance.. like why even have C++ support??????
    motor_func_t motorsReverse;
    motor_func_t motorsTurn;
    motor_func_t motorsForward;

    // Main constructor
    SensorController(pins_t pins, Servo srvo)
    {
      // Set sensor pins
      sensor_pins = pins;

      // Initialize sensor objects  //
      
      // Initialize servo
      servo = srvo;
            
      // Initialize IMU
      imu = new IMU(&serialBus);

      // Initialize ultrasonic sensors
      frontSonicSensor = new UltrasonicSensor(sensor_pins.frontEchoPin, sensor_pins.frontTrigPin);
      frontSonicSensor->offsetDist = 0;
      frontSonicSensor->bufferDist = 5;

      // Initialize line tracker
      lineTracker = new LineTracker(sensor_pins);      

      lineTracker->motorsReverse = motorsReverse;
      lineTracker->motorsTurn = motorsTurn;
      lineTracker->motorsForward = motorsForward;

      monitorBlackLine = false;
    }

    // Check bumpers for collision and handle response
    // TODO: CHECK AND RETURN WHICH BUMPER COLLIDED
    void checkBumpers()
    {
      if(bumpers.checkForCollision())
      {
        unsigned long start = millis();
        
        while(bumpers.checkForCollision())
        {
          //  Play alert
          bumpers.alert(sensor_pins.buzzer);

          // Reverse away from the object
          motorsReverse(frontSonicSensor->bufferDist);
        }
      } else {
        bumpers.setStatusLed(1);
      }
    }

    // Monitor forward sensors
    // Check objects in front of marv and attempt to handle various situations
    // TODO: Determine a way to return necessary data
    void monitorForwardSensors()
    {
      long dist = frontSonicSensor->measure();
      
      // Check if object is within forward buffer zone
      if(dist <= frontSonicSensor->offsetDist + frontSonicSensor->bufferDist)
      {
        // Reverse away from the object
        motorsReverse(frontSonicSensor->bufferDist);
      }
      
      // Check if black line monitoring is on
      if(monitorBlackLine)
      {
        // Check if a sensor was activated
        if(lineTracker->checkForBlackLine(900))
        {
         // Start following black line
         lineTracker->followBlackLine();
        }
      }

      // Check bumpers
      checkBumpers();
      
    }

    // Perform an environmental sweep (assuming the sonic sensor is mounted on top of the servo
  void servoSweep(int startPos, int endPos, int stepSize, long* outputs, uint32_t delayMs=100)
  {
    // Array index (since the loop is using values that may not be from 0-length of array)
    int arrIdx = 0;
    
    // Loop through specified sweep range
    for(int i = startPos; i <= endPos; i++)
    {
      // Move servo and store measurements in return array (outputs)
      servo.write(i);
      delayMicroseconds(100);
      outputs[arrIdx] = frontSonicSensor->measure();
      delayMicroseconds(100);
      arrIdx++;
    }

    // Return servo to face forward
    servo.write(90);
  }
};



#endif  //  End SENSOR_CONTROLLER_H
