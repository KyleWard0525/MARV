#ifndef UTILS_H
#define UTILS_H

/**
 * Utility functions for supporting the marv robot
 * 
 * kward
 */
#include <Math.h>

// Create a custom type definition for MARV's internal functions
typedef void (*marv_func_t)();

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

void alarm(int freq, int buzzer, double dur, int beeps)
{
  // Ensure Green and blue led are low
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, LOW);

  // Play alarm beeps
  for(int i = 0; i < beeps; i++)
  {
    digitalWrite(RED_LED, HIGH);
    beep(freq,buzzer,dur);
    digitalWrite(RED_LED, LOW);
    delay(dur/2);
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
