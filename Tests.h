#ifndef TESTS_H
#define TESTS_H

/**
 * This file contains functions for testing aspects of the
 * system
 */
#include "Marv.h"
#include "Telemetry.h"


// For testing new functionality
void testIMU(Marv* robot)
{
  double arr[3];
  
  robot->imu->getAccels(arr);
  Serial.print("\nAx = " + String(arr[0]) + "g");
  Serial.print("\tAy = " + String(arr[1]) + "g");
  Serial.print("\tAz = " + String(arr[2]) + "g\n");
  delay(1000);
}

void straightLineTest(Marv* robot, double dist, uint32_t sampleRateHz)
{
  // Create an IMU session and set IMU sample rate
  IMU_Session session(500);
  session.sampleRateMs = hertzToMilliseconds(sampleRateHz);  //  Convert Hz to milliseconds

  // Set IMU session
  robot->imu->session = &session;

  // Move
  robot->motors->forward(dist);

  // Write measurements over serial
  Serial.println("<IMU_DATA>");
  Serial.println("sample_rate=" + String(sampleRateHz));
  for(int i = 0; i < session.samples; i++)
  {
      session.data[i].to_string();
  }
  Serial.println("<END>");
}

// Test storing data in an IMU session
void testIMU_Session(Marv* robot)
{
  testIMU(robot);
  delay(500);
  
  // Create an IMU session
  IMU_Session session(500);
  session.sampleRateMs = 25;  //  ms betweeen samples

  // Set IMU session
  robot->imu->session = &session;
  
  unsigned long start = millis();
  
  // Poll IMU and print result stored in imu session data
  robot->motors->driveSpeed = 50;

  // Drive a square
  int dist = 30;
  for(int i = 0; i < 4; i++)
  {
    robot->motors->forward(dist);
    delay(500);
    robot->motors->turn(90);
    delay(500);
  }

  // Write measurements over serial
  Serial.println("<IMU_DATA>");
  for(int i = 0; i < session.samples; i++)
  {
      session.data[i].to_string();
  }
  Serial.println("<END>");
}


// Test resizing internal IMU session data list
void testIMU_SessionResize(Marv* robot)
{
  // Create IMU session
  IMU_Session session(2);
  uint32_t startLen = session.szData;
  
  Serial.println("\nin Tests.testIMUSessionResize(): Session start size: " + String(startLen));

  // Set robot's current IMU session
  robot->imu->session = &session; 
  
  for(int i = 0; i < startLen*4; i++)
  {
    robot->imu->poll();
    delay(50);
    Serial.println("Session size: " + String(session.szData));
  } 
}

// Test the imu_vals_t struct
void testStruct_IMUVals(Marv* robot)
{
  // Create an IMU session
  IMU_Session session(1);
  robot->imu->session = &session;

  // Poll imu
  robot->imu->poll();

  // Get data from session
  imu_vals_t measurement = robot->imu->session->at(0);
  
  double rawAccels[3];
  measurement.getRawAccels(rawAccels);
  double rawGyros[3];
  measurement.getRawGyros(rawGyros);


  Serial.println("\n");
}










#endif  // End TESTS_H
