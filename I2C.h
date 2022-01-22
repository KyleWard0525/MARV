#ifndef I2C_H
#define I2C_H

/**
 * This class is for reading and writing data to and 
 * from memory registers using the board's I2C data 
 * buses
 * 
 * kward
 */
class I2C {
    
  public:

     // Main constructor 
     I2C(){
      // Initialize wire object for serial communication over I2C
      Wire.begin();
  }

    //  Read byte from the device at a given register address
    uint8_t readByte(uint16_t memAddr, uint8_t reg)
    {
      uint8_t regByte;  //  Value of byte read from register
    
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
    uint8_t readBit(uint16_t memAddr, uint8_t regAddr, uint8_t bitPos)
    {
      // Read whole byte from the register
      uint8_t regByte = readByte(memAddr, regAddr);
    
      /*
       * Select the bit by ANDing(clearing) the whole byte
       * except for the one bit at bitPos
       */
      uint8_t bitVal = (regByte & (1 << bitPos));

      // Convert from binary to decimal
      if(bitVal)
      {
        return pow(2,bitPos);
      }
      else {
        return 0;
      }
    }
};


#endif      //  End I2C_H
