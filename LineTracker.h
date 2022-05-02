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

enum PollingType {
  READ_RAW,
  READ_CALIBRATED,
  READ_BLACK_LINE,
  READ_WHITE_LINE
};

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
    
    /**
     * Main constructor
     */
    LineTracker(pins_t periphs)
    {
      pins = periphs;
      setupRSLK();
      clearMinMax(minReadings, maxReadings);
      calibrated = false;
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
      setMotorSpeed(BOTH_MOTORS, 20);

      for(int i = 0; i < random(100,150); i++)
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

    // Follow the black line until it terminates in a "T" intersection
    void followBlackLine()
    {
      // Initialize wheel speeds
      int innerWheelSpeed = 15;
      int outerWheelSpeed = 30;
      
      // Loop until all sensors find a line
      while(!isFull(sensorReadings, LS_NUM_SENSORS))
      {
        // Read the line sensors then read their calibrated values
        readLineSensor(sensorReadings);
        readCalLineSensor(sensorReadings, calibratedReadings, minReadings, maxReadings, DARK_LINE);

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
    }
    
    
    bool isCalibrated()
    {
      return calibrated;
    }
};













#endif  //  End LINE_TRACKER_H
