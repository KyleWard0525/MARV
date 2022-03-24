#ifndef LABS_H
#define LABS_H

/**
 * This file is for storing code from labs 
 * 
 * kward
 */
#include "Marv.h"
#include "Servo.h"

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

/**
 * Lab 6:
 * 
 * The robot will be placed perpendicular to the right of a wall between 15 and 60 degrees.
 * 
 * 1. Turn to find closest point of the wall
 * 2. Drive till 30cm from the wall
 * 3. Rotate 90 degrees clockwise
 * 4. Drive 100cm straight along the wall
 * 
 */
void lab6(Marv* robot)
{
  alarm(300, robot->periphs.buzzer, 500, 1, BLUE_LED);
  int nDeg = 60;
  long distances[nDeg];   //  one element for each degree
  int stopDist = 30;    //  Stop when 30 cm from the wall

  String output = "Distances: ";

  robot->motors->defaultTurnSpeed = 10;
  
  // Loop through each angle
  for(int i = 0; i < 7; i++)
  {
    // Turn 1 degree to the right
    robot->motors->turn(-1);
    
    // Measure distance and store it in array
    distances[i] = robot->frontSonicSensor->measure();

    //output += "Angle = " + String(i+5) + "\t" + "Distance: " + String(distances[i]) + "\n";
    
    delay(1000);
  }
  long closest = 1000;  //  For tracking closest distance while searching for min distance
  long closestIdx = -1; //  For tracking closest index (angle) while searching for min distance


  // Loop through measured distances and search for the smallest
  for(int i = 0; i < nDeg; i++)
  {
    // Check if current distance is closer than current closest
    if(distances[i] < closest)
    {
      // Update closest measurement and corresponding index
      closest = distances[i];
      closestIdx = i;
    }
  }

  if(closestIdx == 0)
  {
    closestIdx += 1;
  }
  delay(1500);
  robot->motors->defaultTurnSpeed = 25;
  // Turn back to be facing wall at the closest point
  robot->motors->turn(nDeg-closestIdx);
  delay(1500);
  
  // Drive until robot is 30cm from the wall
  while(robot->frontSonicSensor->measure() > 40)
  {
    // Drive 5cm forward and display current distance on LED
    robot->motors->forward(1);
    delay(5);
  }

  
  // Assuming the robot is perpendicular to the wall turn to be parallel
  delay(1500);
  robot->motors->turn(120);
  delay(1500);

  // Drive 100cm straight then stop
  robot->motors->forward(100);
}

/**
 * QUIZ 17: 
 *  - Rotate servo motor 180 degrees
 */
void Quiz17(Marv* robot)
{
  Serial.println("in Labs.Quiz17()");
  // Loop for 180 degrees
  for(int i = 0; i <= 180; i++)
  {
    // Turn motor 1 degree then delay
    robot->servo.write(i);
    delay(56);      //  56ms because 10s is 10000ms, 10000/180=55.5556=56
  }
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
