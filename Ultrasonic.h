#ifndef ULTRASONIC_H
#define ULTRASONIC_H
/**
 * This file is meant to be an API for measuring distances using the 
 * HC-SR04 Ultrasonic Sensor
 * kward
 */

class UltrasonicSensor {
  private:
    uint16_t echoPin;
    uint16_t trigPin;
    
 public:
  const uint8_t forwardOffsetDist = 10;   //  Sensor is ~10cm behind front bumpers

  //  Main constructor
  UltrasonicSensor(uint16_t ePin, uint16_t tPin)
  {
    // Set object variables
    echoPin = ePin;
    trigPin = tPin;
  }

  //  Poll distance to nearest object from the sensor (cm)
  long measure()
  {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    /*
     * Set the trigger pin to high for 10 microseconds as the trigger pin
     * is only activated when a 10 microsecond pulse is applied
     */
     digitalWrite(trigPin, HIGH);
     delayMicroseconds(10);
     digitalWrite(trigPin, LOW);

     // Read the amoount of time taken in microseconds for echo pin to recieve a signal
     long duration = pulseIn(echoPin, HIGH);
 
     /*
      * In order to convert the time taken into cm, we must divide the result
      * by 2 then again by 29.1. 
      * 
      * We divide by 2 as the duration contains the amount of time taken
      * for the signal to travel from the target and back but we only want to measure the distance
      * to the target.
      * 
      * Next, we divide by 29.1 because the speed of sound is 340m/s which is approximately
      * 29.1 microseconds per centimeter.
      * 
      */
      return (duration / 2) / 29.1;
  }
};












#endif    //  End ULTRASONIC_H
