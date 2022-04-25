#ifndef UTILS_H
#define UTILS_H

/**
 * Utility functions for supporting the marv robot
 * 
 * kward
 */
#include <Math.h>

LiquidCrystal_I2C lcdI2C_module(0x27,16,2);

// Create a custom type definition for MARV's internal functions
typedef void (*marv_func_t)(void*);

//****    UTILITY DATASTRUCTURES   ****//

// Struct for storing peripherial pins
struct pins_t {
  uint16_t buzzer;          //  Passive buzzer
  uint16_t imuSda;          //  Serial data for IMU
  uint16_t imuClk;          //  Serial clock for IMU
  uint16_t morseLed;        //  LED for morse code blinks
  uint16_t frontTrigPin;    //  Trigger pin for front-facing ultrasonic sensor (signal out)
  uint16_t frontEchoPin;    //  Echo pin for ultrasonic signal    (signal in)
  uint16_t rearTrigPin;     //  Trigger pin for rear-facing ultrasonic sensor (signal out)
  uint16_t rearEchoPin;     //  Echo pin for rear-facing ultrasonic signal    (signal in)
  uint16_t startPin;        //  Button 2
  uint16_t pirPin;          //  PIR Motion Sensor pin
  uint16_t stepIn_1;        //  Input 1 to stepper motor
  uint16_t stepIn_2;        //  Input 2 to stepper motor
  uint16_t stepIn_3;        //  Input 3 to stepper motor
  uint16_t stepIn_4;        //  Input 4 to stepper motor
  uint16_t servoPin;        //  Signal pin for servo motor
  uint16_t lineSesnor_0;    //  Input from the 0th IR phototransistor on the line tracking module 
  uint16_t lineSesnor_1;    //  Input from the 1st IR phototransistor on the line tracking module
  uint16_t lineSesnor_2;    //  Input from the 2nd IR phototransistor on the line tracking module
  uint16_t lineSesnor_3;    //  Input from the 3rd IR phototransistor on the line tracking module
  uint16_t lineSesnor_4;    //  Input from the 4th IR phototransistor on the line tracking module
  uint16_t lineSesnor_5;    //  Input from the 5th IR phototransistor on the line tracking module
  uint16_t lineSesnor_6;    //  Input from the 6th IR phototransistor on the line tracking module
  uint16_t lineSesnor_7;    //  Input from the 7th IR phototransistor on the line tracking module
};

// Struct for storing one measurement of IMU data
struct imu_vals_t {

  unsigned long timepoint;    //  Time since imu/imu session was started
  double Ax;                  //  Lateral acceleration
  double Ay;                  //  Longitudinal acceleration
  double Az;                  //  Vertical acceleration
  double Gx;                  //  Rotational acceleration about the x-axis
  double Gy;                  //  Rotational acceleration about the y-axis
  double Gz;                  //  Rotational acceleration about the z-axis
  double pitch;               //  Current pitch angle
  double roll;                //  Current roll angle
  double heading;             //  Current heading (currently suffers a lot of gyro drift)
  double velocity;            //  Current velocity
  double distance;            //  Distance traveled 

  // Print imu vals
  void to_string()
  {
    Serial.print("Time=" + String(timepoint/1000.0) + ",");
    Serial.print("Ax=" + String(Ax) + ",Ay=" + String(Ay) + ",Az=" + String(Az) + ",");
    Serial.print("Gx=" + String(Gx) + ",Gy=" + String(Gy) + ",Gz=" + String(Gz) + ",");
    Serial.print("Pitch=" + String(pitch) + ",Roll=" + String(roll) + ",");
    Serial.println("Velocity=" + String(velocity) + ",Distance=" + String(distance));
  }

  // Return raw acceleration vector
  void getRawAccels(double* retArr)
  {
    retArr[0] = Ax;
    retArr[1] = Ay;
    retArr[2] = Az;
  }

  // Return raw gyro vector
  void getRawGyros(double* retArr)
  {
    retArr[0] = Gx;
    retArr[1] = Gy;
    retArr[2] = Gz;
  }

};

// Struct for storing sweep data from the sonic-servo module
struct sweep_t {
  int distanceTraveled;
  long measurements[181];

  // Print measurements [servoPos, dist]
  void to_string()
  {
    Serial.print("\nd=" + String(distanceTraveled) + ",[");
    int n = 181;
    
    for(int i = 0; i < n; i++)
    {
      if(i < n - 1)
      {
        Serial.print("[" + String(i) + "," + String(measurements[i]) + "],");
      }
      else {
        Serial.print("[" + String(i) + "," + String(measurements[i]) + "]]\n");
      }
    }
  }
};


//****    UTILITY FUNCTIONS   ****//

// Play a beep sound to the buzzer at a given frequency
void beep(int freq, int buzzer)
{
    digitalWrite(buzzer, 1);
    delayMicroseconds(freq);
    digitalWrite(buzzer, 0);
    delayMicroseconds(freq);
}

// Beep override with duration in ms
void beep(int freq, int buzzer, double dur)
{
  for(int i = 0; i < dur; i++)
  {
    beep(freq, buzzer);
    delay(1);
  }
  
}

// Play alarm using beeper and given led
void alarm(int freq, int buzzer, double dur, int beeps, int led)
{
  // Check led and ensure all others are off
  switch(led) {

    // Red LED will be used
    case RED_LED:
      // Ensure Green and blue led are low
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(BLUE_LED, LOW);
      break;

    // Blue LED will be used
    case BLUE_LED:
      // Ensure red and blue led are low
      digitalWrite(RED_LED, LOW);
      digitalWrite(BLUE_LED, LOW);
      break;

    // Yellow LED will be used
    case YELLOW_LED:
      // Ensure red , green, and blue led are low
      digitalWrite(RED_LED, LOW);
      digitalWrite(BLUE_LED, LOW);
      digitalWrite(GREEN_LED, LOW);
      break;
  }
  

  // Play alarm beeps
  for(int i = 0; i < beeps; i++)
  {
    digitalWrite(led, HIGH);
    beep(freq,buzzer,dur);
    digitalWrite(led, LOW);
    delay(dur/2);
  }
}

// Clamp a variable to be within a specified range of values
int clamp(int value, int minVal, int maxVal)
{
  if(value < minVal)
  {
    return minVal;
  }
  else if(value > maxVal)
  {
    return maxVal;
  }
  else {
    return value;
  }
}

// Apply a low pass filter to an array of doubles.
void lowPassFilter(double *arr, int len, double alpha)
{
    double y = arr[0];
    for (uint32_t i = 1; i < len; i++)
    {
        y = alpha * arr[i] + (1.0 - alpha) * arr[i-1];
        arr[i] = y;
    }
}

// Perform a range check on a value. 0 = not in range, 1 = in range
int range_check(long val, long minVal, long maxVal)
{
  return (val >= minVal) && (val <= maxVal);
}

// Sort an array of longs in ascending order.
void sort(long *a, int n)
{
    int i, j;
    long t;
    for (i = 0; i < n - 1; i++)
    {
        for (j = i + 1; j < n; j++)
        {
            if (a[i] > a[j])
            {
                t = a[i];
                a[i] = a[j];
                a[j] = t;
            }
        }
    }
}

// Find the index of the minimum value in an array of longs.
int minIndex(long *a, int n)
{
    int i;
    long minVal = a[0];
    int minIndex = 0;
    for (i = 1; i < n; i++)
    {
        if (a[i] < minVal)
        {
            minVal = a[i];
            minIndex = i;
        }
    }
    return minIndex;
}

// Convert Hertz to milliseconds
double hertzToMilliseconds(int hertz)
{
  return 1000.0 / hertz;
}

// Convert milliseconds to Hertz
double millisecondsToHertz(int milliseconds)
{
  return (1.0/milliseconds) * 1000.0;
}

// Convert servo position to turn angle
int servoPosToTurnAngle(int servoPos)
{
  return map(servoPos, 0, 180, 90, -90);
}

// Print an array of values
void printArray(long* vals, uint32_t len)
{
  Serial.print("{");

  for(int i = 0; i < len; i++)
  {
    if(i != len-1)
    {
      Serial.print(String(vals[i]) + ", ");
    }
    else {
      Serial.print(String(vals[i]) + "} \n");
    }
  }
}

// Remove an element from an array of longs at a given index
void removeElement(long *a, int n, int index)
{
  for(int i = index; i < n-1; i++)
  {
    a[i] = a[i+1];
  }
}


#endif      //  End UTILS_H
