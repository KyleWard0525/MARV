#ifndef LABS_H
#define LABS_H

/**
 * This file is for storing code from labs 
 * 
 * kward
 */
#include "Marv.h"

/* 
 *  Lab 5 demo
 *  
 *  Drive 75 cm straight
 *  Stop
 *  Turn 90deg left
 *  Drive circumference of circle
 *  Stop
 *  Turn 90deg left
 *  Drive 75cm straight
 *  Stop
 */
void lab5Demo(Marv* robot)
{
  double r = 75.0;
  double c = 2.0 * M_PI * r;
  double cmPerDeg = c / 360.0;

  alarm(500, 2, 500, 3, RED_LED);
  delay(500);

  // Drive radius then turn 90deg counter-clockwise
  robot->motors->forward(r);
  delay(1000);
  robot->motors->turn(-90);
  delay(1000);

  // Drive circumference of circle //

  // Compute number of pulses for each wheel
  //  Inner (left) wheel pulses
  int innerPulses = ((r - robot->motors->getWheelBase()) * M_PI / robot->motors->getWheelDiameter()*M_PI) * robot->motors->getGearRatio() * robot->motors->getPulsesPerMotorRev();
  innerPulses /= 5;
  
  //  Outer (right) wheel pulses
  int outerPulses = ((r + robot->motors->getWheelBase()) * M_PI / robot->motors->getWheelDiameter()*M_PI) * robot->motors->getGearRatio() * robot->motors->getPulsesPerMotorRev();   
  outerPulses /= 5;

  // Reset encoder counts
  resetLeftEncoderCnt();
  resetRightEncoderCnt();

  // Set motor direction and enable motors
  robot->motors->leftMotor.directionForward();
  robot->motors->rightMotor.directionForward();
  
  robot->motors->leftMotor.enableMotor();
  robot->motors->rightMotor.enableMotor();

  // Set motor speed (start moving)
  robot->motors->leftMotor.setSpeed(36);
  robot->motors->rightMotor.setSpeed(43);

  // Wait for motor encoder to read nPulses (distance traveled = dist_cm)
  while((getEncoderLeftCnt() < innerPulses) && (getEncoderRightCnt() < outerPulses))
  {
     delay(1);
  }
 
  // Stop motors
  robot->motors->leftMotor.disableMotor();
  robot->motors->rightMotor.disableMotor();

  delay(1000);
  robot->motors->turn(-90);
  delay(1000);
  robot->motors->forward(r-4);
  delay(1000);
  robot->motors->turn(-180);
}

// Drive square test
void driveSquare(Marv* robot, float sideLen)
{
  alarm(500, 2, 500, 3, RED_LED);
  delay(500);
  
  Serial.println("\nInitial heading: " + String(robot->imu->heading));
  delay(1000);
  robot->motors->forward(sideLen);
  delay(1000);
  robot->motors->turn(-90);
  delay(1000);
  robot->motors->forward(sideLen);
  delay(1000);
  robot->motors->turn(-90);
  delay(1000);
  robot->motors->forward(sideLen);
  delay(1000);
  robot->motors->turn(-90);
  delay(1000);
  robot->motors->forward(sideLen);
  delay(1000);
  robot->motors->turn(-90);
  delay(1000);
}

/**
 * Technically this is Quiz 14 but its basically a lab
 */
long ultraSonicMeasurement(Marv* robot)
{
  // Measure sensor
  long distance = robot->frontSonicSensor->measure();
  Serial.println("Distance: " + String(distance) + " cm");
  return distance;
}





#endif  //  End LABS_H
