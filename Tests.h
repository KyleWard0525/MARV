#ifndef TESTS_H
#define TESTS_H

/**
 * This file contains functions for testing aspects of the
 * system
 */
#include "Marv.h"
#include "Telemetry.h"

// Gradually increase motor speed test
void testSpeedGradIncrease(Marv* robot)
{
  int startSpeed = 5;
  int endSpeed = 250;

  // Set robot's speed parameters
  robot->motors->setStartSpeed(startSpeed);
  robot->motors->driveSpeed = endSpeed;

  // Initiate test
  robot->motors->forward(300);
  delay(1000);

  robot->motors->turn(90);
  delay(500);
  robot->motors->turn(-90);
  delay(500);
}

void testServo(Marv* robot)
{
  robot->servo.write(0);
  delay(500);
  long dist = robot->sensors->frontSonicSensor->measure();
  delay(500);

  robot->lcd->showMessage("R = " + String(dist) + "cm", -1, 0, 0);
  delay(1000);

  robot->motors->turnSpeed = 30;
  robot->motors->turn(-1);
  delay(500);
  robot->servo.write(180);
  delay(500);
  long leftDist = robot->sensors->frontSonicSensor->measure();
  delay(500);
  robot->motors->turn(1);
  delay(500);
  
  robot->lcd->showMessage("L = " + String(leftDist) + "cm", -1, 0, 1);
}

// Test environment localization with the sonic sensor and stepper motor
void testStepperLocalize(Marv* robot)
{
    // Measure front, left, and right distances
    long frontDist = robot->sensors->frontSonicSensor->measure();
    Serial.println("\nFront Distance = " + String(frontDist) + "cm");
    delay(500);

    robot->stepper->turn(-90);
    delay(500);
    long leftDist = robot->sensors->frontSonicSensor->measure();
    Serial.println("Left Distance = " + String(leftDist) + "cm");
    delay(500);

    robot->stepper->turn(190);
    delay(500);
    long rightDist = robot->sensors->frontSonicSensor->measure();
    Serial.println("Right Distance = " + String(rightDist) + "cm");

    long sideLen = rightDist + leftDist;
    long rearDist = sideLen - frontDist;

    Serial.println("Rear distance (assuming within square room): " + String(rearDist) + "cm");
    Serial.println("Side length: " + String(sideLen) + "cm");
}

// Follow black line test
void testFollowBlackLine(Marv* robot)
{
  robot->lcd->showMessage("Press button to", -1, 0, 0);
  robot->lcd->showMessage("calibrate...", -1, 0, 1);
  // Wait for user to initiate sensor calibration
  while(digitalRead(robot->periphs.startPin) == 0)
  {
  
  }
  delay(750);
  robot->lcd->resetScreen();
  robot->lcd->showMessage("Calibrating...", 5000, 1, 0);
}

// For testing new functionality
void testIMU(Marv* robot)
{
  double arr[3];
  
  robot->sensors->imu->getAccels(arr);
  Serial.print("\nAx = " + String(arr[0]) + "g");
  Serial.print("\tAy = " + String(arr[1]) + "g");
  Serial.print("\tAz = " + String(arr[2]) + "g\n");
  delay(1000);
}

// Test telemetry capture in a straight line
void straightLineTest(Marv* robot, double dist, uint32_t sampleRateHz)
{
  // Create an IMU session and set IMU sample rate
  IMU_Session session(5);
  session.sampleRateMs = hertzToMilliseconds(sampleRateHz);  //  Convert Hz to milliseconds

  // Set IMU session
  robot->sensors->imu->session = &session;

  // Move
  robot->motors->forward(dist);

  // Write measurements over serial
  session.writeToSerial();
 
}

// Test writing sonic sweep data over serial
void testSweepPrint(Marv* robot)
{
  uint32_t nSweeps = 3;
  int forwardDist = 8;

  List<sweep_t> sweeps(nSweeps);

  for(int i = 0; i < nSweeps; i++)
  {
    // Create sweep struct
    sweep_t sweep;
    sweep.distanceTraveled = i * forwardDist;

    // Sweep environment
    int endPos = sizeof(sweep.measurements)/sizeof(sweep.measurements[0]);
    robot->sensors->servoSweep(0, endPos, 1, sweep.measurements);

    // Add sweep to list
    sweeps.push(sweep);

    delay(500);
    robot->motors->forward(forwardDist);
    delay(500);
  }

  // Print data to serial
  Serial.println("<SWEEP_DATA>");
  for(int i = 0; i <= nSweeps; i++)
  {
    sweeps.get(i).to_string();
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
  robot->sensors->imu->session = &session;
  
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
      session.data->get(i).to_string();
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
  robot->sensors->imu->session = &session; 
  
  for(int i = 0; i < startLen*4; i++)
  {
    robot->sensors->imu->poll();
    delay(50);
    Serial.println("Session size: " + String(session.szData));
  } 
}

// Test the imu_vals_t struct
void testStruct_IMUVals(Marv* robot)
{
  // Create an IMU session
  IMU_Session session(1);
  robot->sensors->imu->session = &session;

  // Poll imu
  robot->sensors->imu->poll();

  // Get data from session
  imu_vals_t measurement = robot->sensors->imu->session->data->get(0);
  
  double rawAccels[3];
  measurement.getRawAccels(rawAccels);
  double rawGyros[3];
  measurement.getRawGyros(rawGyros);


  Serial.println("\n");
}


// Test servo sweep util function
void testServoSweep(Marv* robot)
{
  
  int minPos = 45;
  int maxPos = 170;
  int stepSize = 1;
  int nPolls = (maxPos - minPos) / stepSize;
  long outputs[nPolls];
  Serial.println("In testServoSweep()");
  robot->sensors->servoSweep(minPos, maxPos, stepSize, outputs);
  Serial.print("\nMeasurements: ");
  printArray(outputs, nPolls);

  // Get index of minimum valiue in outputs
  int minIdx = minIndex(outputs, nPolls);
  long minDist = outputs[minIdx];

  Serial.println("Minimum index: " + String(minIdx));
  Serial.println("Minimum distance: " + String(minDist) + "cm");

  delay(1000);
  int servoPos = map(minIdx, 0, nPolls, minPos, maxPos);
  Serial.println("Servo pos. of minimum index: " + String(servoPos));
  robot->servo.write(servoPos);

  int turnAngle = servoPosToTurnAngle(servoPos);
  Serial.println("Marv angle to move to align with servo: " + String(turnAngle));

  delay(1000);
  robot->motors->turn(turnAngle);
}



#endif  // End TESTS_H
