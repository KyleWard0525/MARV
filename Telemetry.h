#ifndef TELEMETRY_H
#define TELEMETRY_H
/**
 * API for managing various types of telemetry
 * 
 * kward
 */

#include "List.h"



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
    uint32_t szMax;                           //  Max number of elements that can be stored in data

    // Initializer function
    void init()
    {
     // Set session start time
     startTime = millis();
      
     // Set default values for variables
     velocity = 0.0;
     distance = 0.0;
     samples = 0;
     prevTimepoint = 0;

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
    unsigned long prevTimepoint;              //  Timepoint of last measurement
    List<imu_vals_t>* data;                   //  Data list
    
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
      data = new List<imu_vals_t>(szData);
    }

    // Write session data over Serial
    void writeToSerial()
    {
      Serial.println("<IMU_DATA>");
      Serial.println("distance=" + String(round(distance)));
      Serial.println("sample_rate=" + String(round(floor((millisecondsToHertz(sampleRateMs))))));
      for(int i = 0; i < samples; i++)
      {
          data->get(i).to_string();
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


/**
 * A class for representing a 'session' of sonic servo measurements.
 * 
 * 'Session' meaning a task or set of tasks in which X number of sonic servo
 * sweeps will be taken and measurements stored and grouped together
 */
class SonicServo_Session {

  private:
    unsigned long startTime;                  //  Start time for the session
    unsigned long endTime;                    //  End time for the session
    uint32_t szMax;                           //  Max number of sweeps that can be stored in data

    // Internal initializer function
    void init()
    {
      // Set start time
      startTime = millis();

      // Set max size
      szMax = 200;
    }

  public:
    unsigned long prevTimepoint;              //  Timepoint of last measurement
    List<sweep_t>* data;                      //  List of servo sweeps

    // Main constructor
    SonicServo_Session(uint32_t nSweeps)
    {
      // Check if n_size is in range
      if(nSweeps > szMax)
      {
        // Set size of data to be max size
        data = new List<sweep_t>(szMax);
        Serial.println("\n\nWARNING in SonicServo_Session(): given size of " + String(nSweeps) + " > max size = " + String(szMax) + "\nData size set to: " + String(szMax)); 
      }
      else {
        data = new List<sweep_t>(nSweeps);
      }

      
    }
  
};







#endif  //  End TELEMETRY_H
