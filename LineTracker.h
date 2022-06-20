#ifndef LINE_TRACKER_H
#define LINE_TRACKER_H

/**
 * A simplified API for communicating with and controlling the QTR line tracking 
 * sensors
 * 
 * @author kward
 */ 
#include "SimpleRSLK.h"
#include "QTRSensors.h"
#include "Marv.h"



class LineTracker {
  private: 
    pins_t pins;
    bool calibrated;
    uint16_t sensorReadings[LS_NUM_SENSORS];
    uint16_t calibratedReadings[LS_NUM_SENSORS];
    uint16_t maxReadings[LS_NUM_SENSORS];
    uint16_t minReadings[LS_NUM_SENSORS];

  public:
    QTRReadMode readMode;

    enum MODES {
        Idle,
        Calibrating,
        Following
    } mode;

    motor_func_t motorsReverse;
    motor_func_t motorsTurn;
    motor_func_t motorsForward;
    
    /**
     * Main constructor
     */
    LineTracker(pins_t periphs)
    {
      pins = periphs;
      setupRSLK();
      clearMinMax(minReadings, maxReadings);
      calibrated = false;

      // Set mode
      mode = MODES::Idle;
    }

    /*
     * Calibrate the line tracking sensors by driving straight and acquiring min and max 
     * values read from the surronding environment
     */
    void calibrate()
    {
      // Set motors to drive forward 
      setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
      enableMotor(BOTH_MOTORS);
      setMotorSpeed(BOTH_MOTORS, 10);

      for(int i = 0; i < 100; i++)
      {
        // Poll the line tracking sensors
        readLineSensor(sensorReadings);

        // Update the minimum and maximum readings
        setSensorMinMax(sensorReadings, minReadings, maxReadings);
      }

      // Set calibrated flag
      calibrated = true;
      disableMotor(BOTH_MOTORS);
    }

    // Initiate continuous calibration
    void startCalibrating()
    {
      mode = MODES::Calibrating;
      calibrated = false;
    }

    // Stop continuous calibration
    void stopCalibrating()
    {
      mode = MODES::Idle;
      calibrated = true;
    }

    // Poll line sensors for continuous calibration
    void pollSensors()
    {
      // Poll the line tracking sensors
      readLineSensor(sensorReadings);

      // Update the minimum and maximum readings
      setSensorMinMax(sensorReadings, minReadings, maxReadings);
    }

    // Check for a black line
    int checkForBlackLine(int threshold)
    {
      // Initialize raw and calibrated arrays to all zeros
      arrays::clearArray(sensorReadings, 8);
      arrays::clearArray(calibratedReadings, 8);

      // Read the line sensors then read their calibrated values
      readLineSensor(sensorReadings);
      readCalLineSensor(sensorReadings, calibratedReadings, minReadings, maxReadings, DARK_LINE);

      // Check if a sensor was activated
      if(sensorsActivated(calibratedReadings, 8, threshold) >= 1)
      {
        return 1;
      }
      else {
        return 0;
      }
    }

    // Follow the black line until it terminates in a "T" intersection
    void followBlackLine()
    {
      // Initialize wheel speeds
      int innerWheelSpeed = 10;
      int outerWheelSpeed = 20;

      int readingThreshold = 1000;
      
      setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
      enableMotor(BOTH_MOTORS);
      setMotorSpeed(BOTH_MOTORS, innerWheelSpeed);

     arrays::clearArray(calibratedReadings, 8); 
      
      while(sensorsActivated(calibratedReadings, 8, readingThreshold) != 8)
      {
        // Read the line sensors then read their calibrated values
        readLineSensor(sensorReadings);
        readCalLineSensor(sensorReadings, calibratedReadings, minReadings, maxReadings, DARK_LINE);

        // This means the robot left the line (it didn't end with a T)
        if(sensorsActivated(calibratedReadings, 8, readingThreshold) == 0)
        {
          
        }
  
        // Calculate the position of the black line
        uint32_t linePos = getLinePosition(calibratedReadings,DARK_LINE);
        delay(10);
  
        if(linePos > 0 && linePos < 3000) 
        {
          setMotorSpeed(LEFT_MOTOR,innerWheelSpeed);
          setMotorSpeed(RIGHT_MOTOR,outerWheelSpeed);
        } 
        else if(linePos > 3500) 
        {
          setMotorSpeed(LEFT_MOTOR,outerWheelSpeed);
          setMotorSpeed(RIGHT_MOTOR,innerWheelSpeed);
        } 
        else {
          setMotorSpeed(LEFT_MOTOR,innerWheelSpeed);
          setMotorSpeed(RIGHT_MOTOR,innerWheelSpeed);
        }
      }
      disableMotor(BOTH_MOTORS);
   }  

  // Clear raw and calibrated sensor readings
  void clearReadings()
  {
    arrays::clearArray(sensorReadings, 8);
    arrays::clearArray(calibratedReadings, 8);
  }
    
  bool isCalibrated()
  {
    return calibrated;
  }

  void getRawReadings(uint16_t* retArr)
  {
    for(int i = 0; i < LS_NUM_SENSORS; i++)
    {
      retArr[i] = sensorReadings[i];
    }
  }

  void getCalibratedReadings(uint16_t* retArr)
  {
    for(int i = 0; i < LS_NUM_SENSORS; i++)
    {
      retArr[i] = calibratedReadings[i];
    }
  }

  void getMinReadings(uint16_t* retArr)
  {
    for(int i = 0; i < LS_NUM_SENSORS; i++)
    {
      retArr[i] = minReadings[i];
    }
  }

  void getMaxReadings(uint16_t* retArr)
  {
    for(int i = 0; i < LS_NUM_SENSORS; i++)
    {
      retArr[i] = maxReadings[i];
    }
  }
};













#endif  //  End LINE_TRACKER_H
