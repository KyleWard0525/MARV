#ifndef TELEMETRY_H
#define TELEMETRY_H
/**
 * API for managing various types of telemetry
 * 
 * kward
 */

#include "Utils.h"


/**
 * High-level class for managing telemetry data
 */
class Telemetry {

  
};


/**
 * A class for representing a 'session' of IMU data.
 * 
 * 'Session' meaning a time frame and or number of measurements  
 * made by the IMU. An IMU session can range from non-moving ambient measurements,
 * to measuring data through a turn, or even for an entire task / set of tasks.
 */
class IMU_Session {

  private:
    unsigned long startTime;                  //  Start time for the session
    unsigned long endTime;                    //  End time for the session
    unsigned long prevTimepoint;              //  Timepoint of last measurement
    uint32_t currIdx;                         //  Current measurement index
    uint32_t capacity;                        //  Number of elements that can be stored in data

  public:
    uint32_t measurements;                    //  Number of measurements made/will be made
    uint32_t sampleRateMs;                    //  Delay in ms between samples
    uint32_t samples;                         //  Number of samples taken
    double velocity;                          //  Current velocity
    double distance;                          //  Distance traveled
    imu_vals_t* data;                         //  IMU data for each measurement
    
    // Main constructor
    IMU_Session(uint32_t n_measures)
    {
      // Set class variables
      startTime = millis();
      measurements = n_measures;
      data = new imu_vals_t[measurements];
      currIdx = -1;
      velocity = 0;
      distance = 0;
      samples = 0;
      prevTimepoint = 1;
      capacity = measurements;
    }

    // Add IMU data to the end of the data list
    void push(imu_vals_t imu_data)
    {
      // Check if there's enough space in the list
      if(currIdx + 1 < measurements)
      {
        // Update current index and store data
        currIdx++;
        data[currIdx] = imu_data;

        // Update previous timepoint
        prevTimepoint = imu_data.timepoint;
      }
      // List is full
      else {
        // Resize list and try again
        resize();
        currIdx++;
        data[currIdx] = imu_data;
      }
    }

    // Resize internal data array
    void resize()
    {
      uint32_t startSize = capacity;            //  Get size of data
      uint32_t newSize = startSize * 2;         //  Compute new array size

      

      // Create new, larger array
      imu_vals_t newData[newSize];

      // Iterate through data and store in newData
      for(int i = 0; i < startSize; i++)
      {
        // Transfer data from old list to new list
        newData[i] = data[i];
      }

      // Set internal data array to newly sized array
      this->data = newData;

      // Update capacity
      capacity = newSize;
      Serial.println("\nIMU_Session resized to hold " + String(capacity) + " samples");
    }

    // Return length of the list
    uint32_t getCapacity()
    {
      return capacity;
    }

    // Get start time
    unsigned long getStartTime()
    {
      return startTime; 
    }

   // Get time of last measurement
   unsigned long getPrevTimepoint()
   {
    return prevTimepoint;
   }
  
};











#endif  //  End TELEMETRY_H
