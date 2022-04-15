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
    uint32_t szMax;                           //  Max number of elements that can be stored in data

    // Initializer function
    void init()
    {
     // Set session start time
     startTime = millis();
      
     // Set default values for variables
     currIdx = -1;
     velocity = 0;
     distance = 0;
     samples = 0;
     prevTimepoint = 1;

     // Set default sample rate to 80Hz
     sampleRateMs = hertzToMilliseconds(80);

     // Set max list size
     szMax = 500;
    }
    

  public:
    uint32_t szData;                          //  Initial size for data list
    uint32_t sampleRateMs;                    //  Delay in ms between samples
    uint32_t samples;                         //  Number of samples taken
    double velocity;                          //  Current velocity
    double distance;                          //  Distance traveled
    imu_vals_t* data;                         //  IMU data for each measurement
    
    // Main constructor
    IMU_Session(uint32_t n_size)
    { 
      // Initialize
      init();

      // Check if n_size is in range
      if(n_size > szMax)
      {
        // Set size of data to be max size
        szData = szMax;
        Serial.println("\n\nWARNING in IMU_Session(): given size of " + String(n_size) + " > max size = " + String(szMax) + "\nData size set to: " + String(szData)); 
      }
      else {
        // Set initial size
        szData = n_size;
      }
      
      // Initialize internal data array
      data = new imu_vals_t[szData];
    }

    // Add IMU data to the end of the data list
    void push(imu_vals_t imu_data)
    {
      // Check if there's enough space in the list
      if(currIdx + 1 < szData)
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
      uint32_t startSize = szData;            //  Get size of data
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
      data = newData;

      // Update capacity
      szData = newSize;
    }

    // Return the imu_val struct at the corresponding index
    imu_vals_t at(uint32_t idx)
    {
      // Validate index
      if(!range_check(idx, 0, szData-1))
      {
        Serial.println("\n\nERROR in IMU_Session.at(): index " + String(idx) + " out of bounds for list of size " + String(szData));
        exit(0);
      }

      // Return imu_vals
      return data[idx];
    }

    // Write session data over Serial
    void writeToSerial()
    {
      Serial.println("<IMU_DATA>");
      Serial.println("distance=" + String(round(distance)));
      Serial.println("sample_rate=" + String(round(floor((millisecondsToHertz(sampleRateMs))))));
      for(int i = 0; i < samples; i++)
      {
          data[i].to_string();
      }
      Serial.println("<END>");
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
