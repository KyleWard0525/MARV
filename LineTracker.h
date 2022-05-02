#ifndef LINE_TRACKER_H
#define LINE_TRACKER_H

/**
 * A simplified API for communicating with and controlling the QTR line tracking 
 * sensors
 * 
 * @author kward
 */ 
#include "QTRSensors.h"

enum PollingType {
  READ_RAW,
  READ_CALIBRATED,
  READ_BLACK_LINE,
  READ_WHITE_LINE
};

class LineTracker {

  private: 
    pins_t pins;
    QTRSensors lineSensors;
    QTRReadMode defaultReadMode;

    /**
     * Initialize the IR line tracking sensor
     */
    void init()
    {
      // Cast down to 8-bits for QTRSensors API
      const uint8_t ir_pins[8] = {pins.lineSensor_0,pins.lineSensor_1,pins.lineSensor_2,pins.lineSensor_3,
                                  pins.lineSensor_4,pins.lineSensor_5,pins.lineSensor_6,pins.lineSensor_7};
                                  
      //  Initialize internal sensor controller  //

      // Set sensor pins and set type to RC
      lineSensors.setSensorPins(ir_pins, 8);
      lineSensors.setTypeRC();

      // Set emitter pins
      lineSensors.setEmitterPins(pins.lineSensor_Odd, pins.lineSensor_Even);

      // Set default read mode
      defaultReadMode = QTRReadMode::On;

      // Calibrate sensors
      lineSensors.calibrate(defaultReadMode);
    }

  public:
    QTRReadMode readMode;
    
    /**
     * Main constructor
     */
    LineTracker(pins_t periphs)
    {
      pins = periphs;

      // Initialize line tracking module
      init();

      // Set readMode to default
      readMode = defaultReadMode;

      printConfig();
    }

    String sensorType()
    {
      QTRType _type = lineSensors.getType();
      if(_type == QTRType::Undefined)
      {
        return "Undefined";
      }
      else if(_type == QTRType::RC)
      {
        return "RC";
      }
      else {
        return "Analog";
      }
    }

    /**
     * Print current configuration
     */
    void printConfig()
    {
      Serial.println("\nLine Tracker Config:");
      Serial.println("---------------------");
      Serial.println("Sensor type: " + sensorType());
      Serial.println("Emitter pins: " + String(lineSensors.getEmitterPinCount()));
      Serial.println("Dimmable: " + String(lineSensors.getDimmable()));
      Serial.println("Dimming Level: " + String(lineSensors.getDimmingLevel()));
      Serial.println("Timeout duration: " + String(lineSensors.getTimeout()) + "us\n");
    }

    /**
     * Read data from sensors.
     * 
     * Values range from 0 (brightest, highest reflectance) to 1000 (darkest, lowest reflectance)
     */
    void pollSensors(uint16_t* readings, PollingType type)
    {
      // Select polling type
      switch(type)
      {
        case READ_RAW:
          lineSensors.read(readings, readMode);           //  Read raw sensor values
          break;

        case READ_CALIBRATED:
          lineSensors.readCalibrated(readings, readMode); //  Read raw sensor values
          break;

        case READ_BLACK_LINE:
          lineSensors.readLineBlack(readings, readMode);
          break;

        case READ_WHITE_LINE:
          lineSensors.readLineWhite(readings, readMode);

      }

      //  Reverse array so it reads from leftmost sensor (7) to rightmost sensor (0)
      arrays::reverseArray(readings, ARRAY_SIZE(readings));   
    }

    /**
     * Recalibrate sensors
     */
  void recalibrate()
  {
    lineSensors.calibrate(readMode);
  }
};













#endif  //  End LINE_TRACKER_H
