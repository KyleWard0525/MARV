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
typedef void (*marv_func_t)();

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
};

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


// Apply a low-pass filter to the given x,y, and z values
void lowPassFilter(double* forces, double alpha)
{
  // Create filtered variables
  double Fx = 0;
  double Fy = 0;
  double Fz = 0;

  // Apply low-pass filter
  Fx = forces[0] * alpha + ((1.0 - alpha) * Fx);
  Fy = forces[1] * alpha + ((1.0 - alpha) * Fy);
  Fz = forces[2] * alpha + ((1.0 - alpha) * Fz);

  // Return filtered values to array
  forces[0] = Fx;
  forces[1] = Fy;
  forces[2] = Fz;
}





#endif      //  End UTILS_H
