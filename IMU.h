#ifndef IMU_H
#define IMU_H

/**
   This file is for interfacing with the MPU-6050 inertial measurement
   unit

   kward
*/
#include <Wire.h>           //  For communicating with I2C devices
#include <time.h>           //  For timing operations
#include "I2C.h"
#include "Telemetry.h"
#include <MPU6050.h>

class IMU {

  private:
    const byte mpuAddr = 0x68;      //  I2C address for communicating with the MPU-6050 (0x69 if A0 is HIGH)
    const byte slpAddr = 0x6b;      //  I2C address of MPU's power/sleep register
    const byte AxHighAddr = 0x3b;   //  I2C address of X-acceleration's high byte
    const byte AxLowAddr = 0x3c;    //  I2C address of X-accelertation's low byte
    const byte AyHighAddr = 0x3d;   //  I2C address of Y-acceleration's high byte
    const byte AyLowAddr = 0x3e;    //  I2C address of Y-acceleration's low byte
    const byte AzHighAddr = 0x3f;   //  I2C address of Z-acceleration's high byte
    const byte AzLowAddr = 0x40;    //  I2C address of Z-acceleration's low byte
    const byte GxHighAddr = 0x43;   //  I2C address of X-gyro's high byte
    const byte GxLowAddr = 0x44;    //  I2C address of X-gyro's low byte
    const byte GyHighAddr = 0x45;   //  I2C address of Y-gyro's high byte
    const byte GyLowAddr = 0x46;    //  I2C address of Y-gyro's low byte
    const byte GzHighAddr = 0x47;   //  I2C address of Z-gyro's high byte
    const byte GzLowAddr = 0x48;    //  I2C address of Z-gyro's low byte
    const byte whoAmIAddr = 0x75;   //  I2C address for MPU's identifier (READ ONLY REGISTER)
    const byte aConfigAddr = 0x1c;  //  I2C address for acceleration tolerace config register
    const byte gConfigAddr = 0x1B;  //  I2C address for gyro tolerace config register
    const byte srDivAddr = 0x19;    //  I2C address for the sample rate divider register
    uint8_t gyroConfig;             //  Gyrometer configuration data byte
    uint8_t accelConfig;            //  Acceleration configuration data byte
    unsigned long startTime;        //  Time at device startup
    I2C* serialBus;                 //  Interface for communicating with device through I2C
    


  public:
    IMU_Session* session;           //  Current IMU session (if any)
    MPU6050* mpu;                   //  MPU interface
    double heading;                 //  Current heading (deg)
    double velocity;                //  Current velocity (cm/s)
    double distance;                //  Total distance traveled
    unsigned long prevTimeStep;     //  Time since last measurement

    // Main constructor
    IMU(I2C* bus)
    {
      this->serialBus = bus;

      // Ensure MPU address is correct
      if (serialBus->readByte(mpuAddr, whoAmIAddr) != 0x68)
      {
        Serial.println("ERROR in IMU(): invalid MPU address!");
        return;
      }
      else {

        // Initialize MPU6050
        this->mpu = new MPU6050();

        while (!mpu->begin(MPU6050_SCALE_500DPS, MPU6050_RANGE_4G))
        {
          Serial.println("Error: MPU device not found!");
          delay(1000);
        }

        mpu->setAccelPowerOnDelay(MPU6050_DELAY_3MS);
        mpu->setClockSource(MPU6050_CLOCK_INTERNAL_8MHZ);

        // Calibrate gyroscope
        mpu->calibrateGyro();

        start();

        Serial.println("IMU initialized with the following configuration: ");
        getSettings();
      }
    }

    // Start IMU
    void start()
    {
      // Configure device //
    
      // Set initial positional variables to zero
      heading = 0;
      velocity = 0;
      distance = 0;
      prevTimeStep = 1;
      
      /*
         Gyro and accel config registers only use bit 3 and 4 for setting the full scale
         range
      */
      this->gyroConfig = 0x08;  // 0x08 = 000[01]000 = 1 = +-500 deg/s
      this->accelConfig = 0x08; // 0x08 = 000[01]000 = 1 = +-4g


      // Check current configurations
      if (serialBus->readByte(mpuAddr, gConfigAddr) != gyroConfig)
      {
        // Write gyroConfig to gyro config register
        if (serialBus->writeByte(mpuAddr, gConfigAddr, gyroConfig))
        {
          Serial.println("Gyrometer configured.");
        }
      }

      if (serialBus->readByte(mpuAddr, aConfigAddr) != accelConfig)
      {
        // Write gyroConfig to gyro config register
        if (serialBus->writeByte(mpuAddr, aConfigAddr, accelConfig))
        {
          Serial.println("Accelerometer configured.");
        }
      }

      // Set IMU start time
      startTime = millis();
      
      delay(100);
    }


    /**  
     *   Read acceleration data
     *   
     *   Ax = lateral acceleration
     *    - Positive: left turn
     *    - Negative: right turn
     *    
     *   Ay = longitudinal acceleration
     *    - Positive: Braking/Decelerating
     *    - Negative: Accelerating forward
     *    
     *   Az = Vertical acceleration
     *    - Positive: Moving down
     *    - Negative: Moving up (bad for robot)
     *    
     */
    void getAccels(double* retArr)
    {
      /*
         Read raw acceleration values from MPU registers.

         Both bytes will be left-padded with 8 0s in order to make them
         into 16-bit numbers (since they're being stored into an int16_t).

         Since we need the high byte to be the most significant byte, we left shift it by 8
         to get rid of the 0s that were padded.

         Since the low byte is already in place as it too had 8 0s padded to the left making it the LSB, it does not need to be shifted.

         Finally, add the two numbers together to create a signed 16-bit number.
      */
      int16_t AxRaw = (serialBus->readByte(mpuAddr, AxHighAddr) << 8) | serialBus->readByte(mpuAddr, AxLowAddr);  //  Read raw x acceleration
      int16_t AyRaw = (serialBus->readByte(mpuAddr, AyHighAddr) << 8) | serialBus->readByte(mpuAddr, AyLowAddr);  //  Read raw y acceleration
      int16_t AzRaw = (serialBus->readByte(mpuAddr, AzHighAddr) << 8) | serialBus->readByte(mpuAddr, AzLowAddr);  //  Read raw z acceleration

      // Apply a low pass filter to the signals to remove excess noise
      // Create filtered variables
      double Ax_filt = 1;
      double Ay_filt = 1;
      double Az_filt = 1;
      double alpha = 0.995;
      double epsilon = 1e-7;  //  For avoiding mulitplying by zero
    
      // Apply low-pass filter
      Ax_filt = AxRaw * alpha + ((1.0 - alpha) * Ax_filt);
      Ay_filt = AyRaw * alpha + ((1.0 - alpha) * Ay_filt);
      Az_filt = AzRaw * alpha + ((1.0 - alpha) * Az_filt);

      /*
         Since the IMU has been set to use an accel tolerance of +-4g, raw values must be converted
         from LSB to g by dividing by 8192. According to the datasheet: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
      */
      double Ax = Ax_filt / 8192.0;
      double Ay = Ay_filt / 8192.0;
      double Az = Az_filt / 8192.0;
      

      // Store results in return array
      retArr[0] = Ax;
      retArr[1] = Ay;
      retArr[2] = Az;
    }


    /**  
     *   Read acceleration data
     *   
     *   Gx = Rotational acceleration about the x-axis
     *    - Positive: pitching up
     *    - Negative: pitching down
     *    
     *   Gy = Rotational acceleration about the y-axis
     *    - Positive: rolling right
     *    - Negative: rolling left
     *    
     *   Gz = Rotational acceleration about the z-axis
     *    - Positive: yawing right
     *    - Negative: yawing left
     *    
     */
    void getGyros(double* retArr)
    {
      /*
          Since the MPU6050 uses signed 16-bit numbers for all data registers,
          reading MSB and LSB from the the registers is done in the same way as getAccels()
      */
      int16_t GxRaw = (serialBus->readByte(mpuAddr, GxHighAddr) << 8) | serialBus->readByte(mpuAddr, GxLowAddr);  //  Read raw x acceleration
      int16_t GyRaw = (serialBus->readByte(mpuAddr, GyHighAddr) << 8) | serialBus->readByte(mpuAddr, GyLowAddr);  //  Read raw y acceleration
      int16_t GzRaw = (serialBus->readByte(mpuAddr, GzHighAddr) << 8) | serialBus->readByte(mpuAddr, GzLowAddr);  //  Read raw z acceleration

      // Apply a low pass filter to the signals to remove excess noise
      // Create filtered variables
      double Gx_filt = 1;
      double Gy_filt = 1;
      double Gz_filt = 1;
      double alpha = 0.99;
      double epsilon = 1e-7;  //  For avoiding mulitplying by zero
    
      // Apply low-pass filter
      Gx_filt = GxRaw * alpha + ((1.0 - alpha) * Gx_filt);
      Gy_filt = GyRaw * alpha + ((1.0 - alpha) * Gy_filt);
      Gz_filt = GzRaw * alpha + ((1.0 - alpha) * Gz_filt);
      
      /*
       * According to the datasheet (https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf),
       * to convert from LSB -> deg/s at a range of +-500 deg/s. We must divide the raw values by 65.5
       */
       double Gx = Gx_filt / 65.5;
       double Gy = Gy_filt / 65.5;
       double Gz = Gz_filt / 65.5;

       retArr[0] = Gx;
       retArr[1] = Gy;
       retArr[2] = Gz;
    }


    // Get device's sample rate
    uint16_t getSamplingRate()
    {
      uint16_t gyroRate = 8000; // 8kHz if DLPF_CFG register is (0 or 7), 1kHz otherwise
      uint8_t sampleRateDiv = serialBus->readByte(mpuAddr, 0x19); // 0x19 is the sample rate divder register

      return gyroRate / (1 + sampleRateDiv);
    }


    /**
     * Compute pitch and roll from given accel values
     * 
     * accels = [Ax, Ay, Az]
     * 
     * Returns:
     *  retArr = [pitch, roll]
     */
    void computePitchRoll(double* accels, double* retArr)
    {
      retArr[0] = (atan2(accels[1], accels[2]) * 180.0) / M_PI;
      retArr[1] = (atan2(accels[0], sqrt(accels[1]*accels[1]+accels[2]*accels[2])) * 180.0) / M_PI;
    }
    


    // Poll values from mpu. Returns [Ax,Ay,Az,Gx,Gy,Gz]
    void poll(double* retArr)
    {
      // Arrays to store accel and gyro values
      double accelArr[3];

      // Read accel values from mpu registers
      getAccels(accelArr);

      // Populate return array with accel values
      retArr[0] = accelArr[0];  //  Ax
      retArr[1] = accelArr[1];  //  Ay
      retArr[2] = accelArr[2];  //  Az

      // Read gyro values from mpu registers
      double gyroArr[3];
      getGyros(gyroArr);

      // Populate return array with gyro values
      retArr[3] = gyroArr[0];   //  Gx
      retArr[4] = gyroArr[1];   //  Gy
      retArr[5] = gyroArr[2];   //  Gz


      // Compute change in time since last measurement
      unsigned long dt = (millis() - prevTimeStep) / 1000;
         
      // Update derivatives
      heading += (gyroArr[2]/500) * dt;
      velocity += accelArr[1] * dt;
      distance += velocity * dt;

      prevTimeStep = dt;
    }

    // Poll values from IMU and store them in an IMU session
    void poll()
    {
      // Read accel values from mpu registers
      double accelArr[3];
      getAccels(accelArr);

      // Compute pitch and roll
      double pitchRoll[2];
      computePitchRoll(accelArr, pitchRoll);

      // Struct for storing data
      imu_vals_t data;
      data.timepoint = millis() - session->getStartTime();

      // Compute change in time since last measurement
      unsigned long dt = (data.timepoint - session->getPrevTimepoint()) / 1000.0;
         
      // Update session derivatives
      //heading += (gyroArr[2]/500) * dt;
      session->velocity += (accelArr[1] * dt) * 32.17; // Convert to feet per second
      session->distance += session->velocity * dt; // In feet

      // Read gyro values from mpu registers
      double gyroArr[3];
      getGyros(gyroArr);

      // Populate data struct and add to session
      data.Ax = accelArr[0];
      data.Ay = accelArr[1];
      data.Az = accelArr[2];
      data.Gx = gyroArr[0];
      data.Gy = gyroArr[1];
      data.Gz = gyroArr[2];
      data.pitch = pitchRoll[0];
      data.roll = pitchRoll[1];
      data.velocity = session->velocity;
      data.distance = session->distance;

      // Ensure a valid measurement (MARV should never fly)
      if(data.Az > 0)
      {
        // Add IMU metrics to list
        session->push(data);
  
        // Increment total samples taken
        session->samples++;
      }
    }


    // Clear values used to integrate relative motion data
    void clear()
    {
       // Reset variables
       heading = 0;
       velocity = 0;
       distance = 0;
       prevTimeStep = 1;
       
    }
    

    // Get IMU start time
    unsigned long getStartTime()
    {
      return startTime;
    }


    /*
       Print MPU configuration settings

       Original author: MissionCritical (Instructables)
       https://www.instructables.com/MPU-6050-Tutorial-How-to-Program-MPU-6050-With-Ard/

       Modified by: kward
    */
    void getSettings()
    {
      Serial.println();

      Serial.print(" * Sleep Mode:        ");
      Serial.println(mpu->getSleepEnabled() ? "Enabled" : "Disabled");

      Serial.print(" * Clock Source:      ");
      switch (mpu->getClockSource())
      {
        case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
        case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
        case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
        case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
        case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
        case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
        case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
      }

      Serial.print(" * Gyroscope:         ");
      switch (mpu->getScale())
      {
        case MPU6050_SCALE_2000DPS:        Serial.println("2000 dps"); break;
        case MPU6050_SCALE_1000DPS:        Serial.println("1000 dps"); break;
        case MPU6050_SCALE_500DPS:         Serial.println("500 dps"); break;
        case MPU6050_SCALE_250DPS:         Serial.println("250 dps"); break;
      }

      // Read accel config register
      uint8_t aConfigByte = serialBus->readByte(mpuAddr, aConfigAddr);
      uint8_t afsBits[2] = {serialBus->readBit(aConfigByte, 4), serialBus->readBit(aConfigByte, 3)};
      uint8_t afsSel = 0;
      uint8_t accelRange = 0;

      // Compute acceleration range in G from config bits
      if(afsBits[0])
      {
        afsSel += afsBits[0] / pow(2, 4);
      }
      if(afsBits[1])
      {
        afsSel += afsBits[1] / pow(2, 3);
      }

      // Check AFS_SEL value and map it to its corresponding G value
      switch(afsSel) {

        // +- 2g
        case 0:
          accelRange = 2;
          break;

        // +- 4g
        case 1: 
          accelRange = 4;
          break;

       // +- 8g
       case 2:
        accelRange = 8;
        break;

      // +- 16g
      case 3:
       accelRange = 16;
       break;
      }
      Serial.println(" * Accelerometer range: +- " + String(accelRange) + "g");
      
      
      Serial.print(" * Gyroscope offsets: ");
      Serial.print(mpu->getGyroOffsetX());
      Serial.print(" / ");
      Serial.print(mpu->getGyroOffsetY());
      Serial.print(" / ");
      Serial.println(mpu->getGyroOffsetZ());

      Serial.println();
    }

    
};



#endif      //  End IMU_H
