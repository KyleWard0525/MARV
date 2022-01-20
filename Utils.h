#ifndef UTILS_H
#define UTILS_H

/**
 * Utility functions for supporting the marv robot
 */

// Play a beep sound to the buzzer at a given frequency
void beep(int freq, int buzzer)
{
    digitalWrite(buzzer, 1);
    delayMicroseconds(freq);
    digitalWrite(buzzer, 0);
    delayMicroseconds(freq);
}











#endif      //  End UTILS_H
