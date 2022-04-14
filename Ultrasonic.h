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
  uint8_t warningLimit = 3;               //  Number of times forward collision is avoided before emitting a warning
  uint8_t alarmLimit = 6;                 //  Number of times forward collision is avoided before playing an alarm            
  int eventDuration = 5000;               //  Duration in ms of each measurement 'event' (i.e. the period of time in which to count the number of collision avoidances)
  int avoids = 0;                         //  Number of forward collisions avoided
  uint32_t offsetDist;                    //  Offset distance from sensor to edge of the robot
  uint32_t bufferDist;                    //  Safe buffer zone to avoid crashing into things  
  long eventStart;                        //  Time at start of current 'event'

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
    long distances[6];

    for(int i = 0; i < 6; i++)
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
        distances[i] = (duration / 2) / 29.1;
    }
    
    // Sort distances and return median
    sort(distances, sizeof(distances)/sizeof(distances[0]));
    return distances[2];
  }

};












#endif    //  End ULTRASONIC_H
