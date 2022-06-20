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

// Create a callback type for motor functions
typedef void(*motor_func_t)(int);

typedef unsigned int uint_t;


#define ARRAY_SIZE(x) sizeof(x)/sizeof(x[0])



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
  uint16_t lineSensor_0;    //  Input from the 0th IR phototransistor on the line tracking module 
  uint16_t lineSensor_1;    //  Input from the 1st IR phototransistor on the line tracking module
  uint16_t lineSensor_2;    //  Input from the 2nd IR phototransistor on the line tracking module
  uint16_t lineSensor_3;    //  Input from the 3rd IR phototransistor on the line tracking module
  uint16_t lineSensor_4;    //  Input from the 4th IR phototransistor on the line tracking module
  uint16_t lineSensor_5;    //  Input from the 5th IR phototransistor on the line tracking module
  uint16_t lineSensor_6;    //  Input from the 6th IR phototransistor on the line tracking module
  uint16_t lineSensor_7;    //  Input from the 7th IR phototransistor on the line tracking module
  uint16_t lineSensor_Even; //  Control even IR sensors pin
  uint16_t lineSensor_Odd;  //  Control odd IR sensors pin
  uint16_t bumper_0;        //  Bumper 0 pin
  uint16_t bumper_1;        //  Bumper 1 pin
  uint16_t bumper_2;        //  Bumper 2 pin
  uint16_t bumper_3;        //  Bumper 3 pin
  uint16_t bumper_4;        //  Bumper 4 pin
  uint16_t bumper_5;        //  Bumper 5 pin
  uint16_t rLed;            //  Onboard red LED
  uint16_t gLed;            //  Onboard green LED
  uint16_t bLed;            //  Onboard blue LED 
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
      digitalWrite(GREEN_LED, LOW);
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

int sign(int x)
{
  if(x < 0)
  {
    return -1;
  }
  else if( x > 0)
  {
    return 1;
  }
  else {
    return 0;
  }
}

// Return the number of line sensors activated within a certain threshold
int sensorsActivated(uint16_t* readings, int n, int threshold)
{
  int activations = 0;

  // Loop through sensor readings
  for(int i = 0; i < n; i++)
  {
    // Check if reading is at or above threshold
    if(readings[i] >= threshold)
    {
      activations++;
    }
  }

  return activations;
}

namespace arrays {
  
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

int maxIndex(long *a, int n)
{
  int maxVal = a[0];
  int maxIndex = 0;
  for (int i = 1; i < n; i++)
  {
    if (a[i] > maxVal)
    {
      maxVal = a[i];
      maxIndex = i;
    }
  }
  return maxIndex;
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
  
  // Print an array of values
  void printArray(uint16_t* vals, uint_t len)
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
  
  // Reverse an array of uint16_t
  void reverseArray(uint16_t *a, int n)
  {
    for(int i = 0; i < n/2; i++)
    {
      uint16_t temp = a[i];
      a[i] = a[n-i-1];
      a[n-i-1] = temp;
    }
  }
  
  // Clear an array of ints
  void clearArray(uint16_t *a, int n)
  {
    for(int i = 0; i < n; i++)
    {
      a[i] = 0;
    }
  }
  
  // Check if an int array is empty
  bool isEmpty(uint16_t *a, int n)
  {
    for(int i = 0; i < n; i++)
    {
      if(a[i] != 0)
      {
        return false;
      }
    }
    return true;
  }
  
  // Check if an int array is full
  bool isFull(uint16_t *a, int n)
  {
    for(int i = 0; i < n; i++)
    {
      if(a[i] == 0)
      {
        return false;
      }
    }
    return true;
  }
  
  // Check number of elements in an array of ints
  int numElements(uint16_t *a, int n)
  {
    int count = 0;
    for(int i = 0; i < n; i++)
    {
      if(a[i] != 0)
      {
        count++;
      }
    }
    return count;
  }

  // Get the index of the largest value in an array of ints
  int maxIndex(uint16_t *a, int n)
  {
    int maxVal = a[0];
    int maxIndex = 0;
    for (int i = 1; i < n; i++)
    {
      if (a[i] > maxVal)
      {
        maxVal = a[i];
        maxIndex = i;
      }
    }
    return maxIndex;
  }

}


#endif      //  End UTILS_H
