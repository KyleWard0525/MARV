
#ifndef UTILS_H
#define UTILS_H

/**
 * Utility functions for supporting the marv robot
 * 
 * kward
 */
#include <Wire.h>
#include <Math.h>

// Play a beep sound to the buzzer at a given frequency
void beep(int freq, int buzzer)
{
    digitalWrite(buzzer, 1);
    delayMicroseconds(freq);
    digitalWrite(buzzer, 0);
    delayMicroseconds(freq);
}

//  Read byte from a register
uint8_t readByte(uint16_t memAddr, uint8_t reg)
{
  uint8_t regByte = 0;  //  Value of byte read from register

  // Open I2C communication with device
  Wire.beginTransmission(memAddr);

  // Select register to read from
  Wire.write(reg);
  
  // End current communication (to allow for another instruction to be sent)
  Wire.endTransmission();

  // Open another comm channel with device and read only the selected register
  Wire.beginTransmission(memAddr);
  
  Wire.requestFrom(memAddr, 1);  //  Tell device to only read the first byte

  // Read byte and end transmission
  regByte = Wire.read();
  Wire.endTransmission();

  return regByte;
}

//  Read bit value from device register at given position
uint8_t readBit(uint16_t memAddr, uint8_t regAddr, int8_t bitPos)
{
  // Read whole byte from the register
  uint8_t regByte = readByte(memAddr, regAddr);

  /*
   * Select the bit by ANDing(clearing) the whole byte
   * except for the one bit at bitPos
   */
  uint8_t bitVal = (regByte & (1 << bitPos));

  if(bitVal)
  {
    return pow(2,bitPos);
  }
  else {
    return 0;
  }
}





#endif      //  End UTILS_H
