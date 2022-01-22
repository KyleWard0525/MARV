#ifndef IMU_H
#define IMU_H

/**
 * This file is for interfacing with the MPU-6050 inertial measurement
 * unit
 * 
 * kward
 */
#include <Wire.h>           //  For communicating with I2C devices
#include "I2C.h"

class IMU {

  private: 
    const uint16_t mpuAddr = 0x68;      //  I2C address for communicating with the MPU-6050 (0x69 if A0 is HIGH)
    const uint16_t slpAddr = 0x6B;      //  I2C address of MPU's power/sleep register
    const uint16_t startAddr = 0x3B;    //  I2C address to start reading values from (0x3B = ACCEL_XOUT_H)
    const uint16_t whoAmIAddr = 0x75;   //  I2C address for MPU's identifier (READ ONLY REGISTER)
    const uint16_t aConfigAddr = 0x1C;  //  I2C address for acceleration tolerace config register
    const uint16_t gConfigAddr = 0x1B;  //  I2C address for gyro tolerace config register
    I2C* serialBus;                     //  Interface for communicating with device through I2C
  public:

    // Main constructor
    IMU(I2C* bus)
    { 
      this->serialBus = bus;
      
      // Ensure MPU address is correct
      if(serialBus->readByte(mpuAddr, whoAmIAddr) != 0x68)
      {
        Serial.println("ERROR in IMU(): invalid MPU address!");
        return;
      }
      else {
        Serial.println("MPU Identifier: " + String(serialBus->readByte(mpuAddr, whoAmIAddr)));
      }

      
    }

    // Start IMU
    void start()
    {
      // Wake up MPU
       Wire.beginTransmission(mpuAddr);
       Wire.write(slpAddr); // Sleep register
       Wire.write(1);
       Wire.endTransmission();
    }

    // Get device's sample rate
    uint16_t getSamplingRate()
    {
      uint16_t gyroRate = 8000; // 8kHz if DLPF_CFG register is (0 or 7), 1kHz otherwise
      uint8_t sampleRateDiv = serialBus->readByte(mpuAddr, 0x19); // 0x19 is the sample rate divder register

      return gyroRate / (1 + sampleRateDiv);
    }
};



#endif      //  End IMU_H
