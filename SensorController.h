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


class SensorController {

  private:
    pins_t sensor_pins;     //  GPIO pins for the sensors

  public:
    I2C serialBus;                          //  I2C API
    IMU* imu;                               //  IMU interface
    UltrasonicSensor* frontSonicSensor;     //  Forward facing sensor for measuring distance using ultrasound
    UltrasonicSensor* rearSonicSensor;      //  Rear facing sensor for measuring distance using ultrasound
    Bumpers bumpers;                        //  Forward facing bump sensors
    Servo servo;                            //  Servo API for front sonic sensor assembly

    // Main constructor
    SensorController(pins_t pins)
    {
      // Set sensor pins
      sensor_pins = pins;

      // Initialize sensor objects  //
      
      // Initialize IMU
      imu = new IMU(&serialBus);

      // Initialize ultrasonic sensors
      frontSonicSensor = new UltrasonicSensor(sensor_pins.frontEchoPin, sensor_pins.frontTrigPin);
      frontSonicSensor->offsetDist = 0;
      frontSonicSensor->bufferDist = 5;

      // Initialize servo
      servo.attach(sensor_pins.servoPin);
    }
  
};


















#endif  //  End SENSOR_CONTROLLER_H
