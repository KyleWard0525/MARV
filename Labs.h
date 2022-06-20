#ifndef LABS_H
#define LABS_H

/**
 * This file is for storing code from labs 
 * 
 * kward
 */
#include "Marv.h"
#include "Servo.h"
#include "List.h"

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

  robot->motors->turnSpeed = 10;
  
  // Loop through each angle
  for(int i = 0; i < 7; i++)
  {
    // Turn 1 degree to the right
    robot->motors->turn(-1);
    
    // Measure distance and store it in array
    distances[i] = robot->sensors->frontSonicSensor->measure();

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
  robot->motors->turnSpeed = 25;
  // Turn back to be facing wall at the closest point
  robot->motors->turn(nDeg-closestIdx);
  delay(1500);
  
  // Drive until robot is 30cm from the wall
  while(robot->sensors->frontSonicSensor->measure() > 40)
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
 * Lab 7:
 * 
 * The robot will be placed in a position such that the closest wall is
 * 120cm away. The robot will:
 * 
 * 1. Turn the servo 180deg taking an ultrasonic measurement every n degrees
 * 2. Store measurements in array
 * 3. Turn 180deg in place
 * 4. Repeat steps 1 and 2
 * 5. Search array for the closest object
 * 6. Turn in place to face closest object
 * 7. Drive towards object
 * 8. Stop when bumpers collide with object
 */
 void lab7(Marv* robot)
 {
  long sweep1[185];     //  Array of distances for first sweep
  long sweep2[185];     //  Array of distances for second sweep

  // Move servo through first sweep
  for(int i = 0; i < (sizeof(sweep1)/sizeof(sweep1[0])); i++)
  {
    // Move servo to position i
    robot->servo.write(i);
    
    // Measure distance and store in sweep1 array
    sweep1[i] = robot->sensors->frontSonicSensor->measure();
    
    delay(50);
  }

  // Find the index of the minimum value in sweep1
  int minIdx1 = minIndex(sweep1, sizeof(sweep1)/sizeof(sweep1[0]));

  // Turn robot 180
  delay(500);
  robot->motors->turn(185);

  // Move servo through second sweep
  for(int i = 0; i < (sizeof(sweep2)/sizeof(sweep2[0])); i++)
  {
    // Move servo to position i
    robot->servo.write(i);
    delay(50);
    
    // Measure distance and store in sweep2 array
    sweep2[i] = robot->sensors->frontSonicSensor->measure();
  }

  // Find index of the minimum value in sweep2
  int minIdx2 = minIndex(sweep2, sizeof(sweep2)/sizeof(sweep2[0]));

  long minDist = 0;

  // Check which sweep the closest object is in
  if(sweep1[minIdx1] < sweep2[minIdx2])
  {
    delay(500);
    // Object is in first sweep so turn back around
    robot->motors->turn(190);
    delay(500);
    
    // Get distance to closest object
    minDist = sweep1[minIdx1];

    // Move servo to point at the closest object
    robot->servo.write(minIdx1);
    
    // Slow motors for more precise turn (?)
    robot->motors->turnSpeed = 40;
    delay(1000);

    // Check which quadrant closest object is in
    if(minIdx1 <= 90)
    {
      // Quadrant 1
      int turnAngle = 90 - minIdx1 - 10;         //  Convert servo position to nDegrees for MARV to turn
      robot->motors->turn(turnAngle);       //  Turn robot to face closest object
      delay(1000);
    }
    else {
      // Quadrant 2
      int turnAngle = -(minIdx1 - 90) + 10;      //  Convert servo position to nDegrees for MARV to turn
      robot->motors->turn(turnAngle);       //  Turn robot to face closest object 
      delay(1000);
    }

    // Turn servo to face forward
    robot->servo.write(90);

    robot->lcd->showMessage("Sweep 1 d=" + String(minDist) + "cm", -1, 0, 0);
    robot->lcd->showMessage("Servo Pos.: " + String(minIdx1), -1, 0, 1);
  }
  // Closest object is in the second sweep (no need to turn around again)
  else {
    // Get distance to closest object
    minDist = sweep2[minIdx2];

    // Move servo to point at the closest object
    robot->servo.write(minIdx2);

    // Slow motors for more precise turn (?)
    robot->motors->turnSpeed = 40;
    delay(1000);

    // Check which quadrant closest object is in
    if(minIdx2 <= 90)
    {
      // Quadrant 1
      int turnAngle = 90 - minIdx2 - 10;    //  Convert servo position to nDegrees for MARV to turn
      robot->motors->turn(turnAngle);       //  Turn robot to face closest object
      delay(1000);
    }
    else {
      // Quadrant 2
      int turnAngle = -(minIdx2 - 90) + 10; //  Convert servo position to nDegrees for MARV to turn
      robot->motors->turn(turnAngle);       //  Turn robot to face closest object 
      delay(1000);
    }

    // Turn servo to face forward
    robot->servo.write(90);
    
    robot->lcd->showMessage("Sweep 2, d=" + String(minDist) + "cm", -1, 0, 0);
    robot->lcd->showMessage("Servo Pos.: " + String(minIdx2), -1, 0, 1);
  }

  delay(500);
  // Move forward until robot touches the wall
  robot->motors->forward(minDist);
 }




/**
 * Lab 8 Part 1: 
 * 
 * The robot will travel down a hallway (arena) which is 2.4m wide and 
 * 3m long. 
 * 
 * There will be 3 30cm^3 boxes placed along the path on either side.
 * Every 20cm, stop and sweep the servo and sonic sensor 180deg and store measurements 
 * 
 * These measurements will later be put into the report in the form of a table
 */
void lab8Part1(Marv* robot)
{
  int totalDist = 300; //  3m
  int measureInc = 20;  // Increments of 20cm between measurements

  // Divide the total path into sections relative to the measurement interval
  int sections = totalDist / measureInc;

  // Create a list of sweeps to store all measurement data
  List<sweep_t> sweeps(sections);

  // Loop through all the 20cm-sections of the hallway
  for(int i = 0; i < sections; i++)
  {
    // Create struct to store sweep data
    sweep_t sweep;
    
    // Measure and store data from sonic sensor
    sweep.distanceTraveled = i * measureInc;  //  Total distance traveled at this step
    int endPos = sizeof(sweep.measurements)/sizeof(sweep.measurements[0]);
    robot->sensors->servoSweep(0, endPos, 1, sweep.measurements);

    // Push sweep to list of sweeps
    sweeps.push(sweep);

    // Move robot forward 20cm 
    robot->motors->forward(measureInc);
  }

  // Print sweep data
  for(int i = 0; i < sweeps.len()+1; i++)
  {
    sweeps.get(i).to_string();
  }
  
}

/**
 * Lab 8 Part 2: 
 * 
 * The robot will travel down a hallway (arena) which is 2.4m wide and 
 * 3m long. 
 * 
 * There will be 3 30cm^3 boxes placed along the path on either side.
 * Every 20cm, stop and sweep the servo and sonic sensor 180deg and store measurements 
 * 
 * When the robot reaches the end of the hallway (3 meters), it should: 
    1. Rotate the robot in place 180-degrees, 
    2.  turn the servo to point forward, 
    3.  travel back down the hallway  
    4.  while returning to the starting point, do the following: 
    a.  stop next to each obstacle 
    b.  turn the servo towards the object 
    c.  wait 3 seconds 
    d.  turn the servo forward 
    e.  start moving again towards the starting point 
 */
struct obj_location_t {
  int path_distance;        //  Distance traveled 
  int servoPos;             //  Servo position 
  long sonic_distance;      //  Distance measured by the ultrasonic sensor
};

void lab8Part2(Marv* robot)
{
  int totalDist = 300; //  3m (CHANGE BACK TO 300CM AFTER TESTING)
  int measureInc = 20;  // Increments of 20cm between measurements

  // Divide the total path into sections relative to the measurement interval
  int sections = totalDist / measureInc;

  // Create a list of sweeps to store all measurement data
  List<sweep_t> sweeps(sections);



  //*** Step 1: Travel down the hallway taking measurements ***//

  
  // Loop through all the 20cm-sections of the hallway
  for(int i = 0; i < sections; i++)
  {
    // Create struct to store sweep data
    sweep_t sweep;
    
    // Measure and store data from sonic sensor
    sweep.distanceTraveled = i * measureInc;  //  Total distance traveled at this step
    int endPos = sizeof(sweep.measurements)/sizeof(sweep.measurements[0]);
    robot->sensors->servoSweep(0, endPos, 1, sweep.measurements);

    // Move robot forward 20cm 
    robot->motors->forward(measureInc);

    // Push sweep to list of sweeps
    sweeps.push(sweep);
  }




 //*** Step 2: Rotate 180 and compute location of objects ***//


  // Rotate robot 180
  delay(1000);
  robot->motors->turn(190);
  delay(1000);

  // Find objects on left and right side (left = measurements[0:89], right = measurements[90:179])
  // Note: left and right are opposite of lab8Part1() 
  obj_location_t objects[3];          // Structs for storing object locations

  
  long minIndices[sections];  // Indices of all closest sonic distances measured (NOTE: each index corresponds to the servo position at which the measurement was made)
  long minDistances[sections];// Parallel (corresponding) array of those distance values

  // Loop through sweeps
  for(int i = 0; i < sections; i++)
  {
    // Get sweep data from list
    sweep_t sweep = sweeps.get(i);
    
    // Find the index (servo position) of the minumum distance in each sweep
    minIndices[i] = minIndex(sweep.measurements, sizeof(sweep.measurements)/sizeof(sweep.measurements[0]));
    minDistances[i] = sweep.measurements[minIndices[i]];
  }

  // Pick the 3 closest object detected
  // NOTE: This means objects are originally sorted by sonic distance, in ascending order
  for(int i = 0; i < 3; i++)
  {
    // Get the index of the sweep with the current minimum sonic distance measured
    int sweepIdx = minIndex(minDistances, sections);

    // Get the minimum sonic distance measured from that sweep
    long sonicDist = minDistances[sweepIdx];

    // Get the position of the servo at which the minimum distance was measured
    int servoPos = minIndices[sweepIdx];

    // Store data about the object 
    objects[i].servoPos = servoPos;
    objects[i].sonic_distance = sonicDist;
    objects[i].path_distance = sweepIdx * measureInc; // Distance robot has traveled at this point in time

    // Set the minimum distance value of this sweep to no longer be the minimum (so we can find the next closest object with the next loop iteration)
    minDistances[sweepIdx] = 9999999;
  }

  
  // Sort objects based on path distance (highest to lowest bc we're traveling in the opposite direction)
  for(int i = 0; i < 2; i++)
  {
    for(int j = i+1; j < 3; j++)
    {
      if(objects[i].path_distance < objects[j].path_distance)
      {
        // Swap objects
        obj_location_t temp = objects[i];
        objects[i] = objects[j];
        objects[j] = temp;
      }
    }
   }



  //*** Step 3: Drive to each object and point the servo at it ***//


  // Check whether or not the last (now first) object was at the end
  if(objects[0].path_distance < totalDist)
  {
    // Move the robot to the first object
    robot->motors->forward(totalDist - objects[0].path_distance);
  }

  if(objects[0].servoPos < 90)
  {
    robot->servo.write(0);
  }
  else {
    // Point servo at the first object and wait 3s
    robot->servo.write(180);
  }
  
  delay(3000);
  robot->servo.write(90);        // Return servo to face forward
  delay(400);

  // Move to the next object
  robot->motors->forward(objects[0].path_distance - objects[1].path_distance);

   if(objects[1].servoPos < 90)
  {
    robot->servo.write(0);
  }
  else {
    // Point servo at the first object and wait 3s
    robot->servo.write(180);
  }

  delay(3000);
  robot->servo.write(90);        // Return servo to face forward
  delay(400);

  // Move to the last object
  robot->motors->forward(objects[1].path_distance - objects[2].path_distance);

  if(objects[2].servoPos < 90)
  {
    robot->servo.write(180);
  }
  else {
    // Point servo at the first object and wait 3s
    robot->servo.write(0);
  }

  delay(3000);
  robot->servo.write(90);        // Return servo to face forward
  delay(400);

  // Check if the last object was at the start/finish line
  if(objects[2].path_distance > 0)
  {
    // Move the last X cm to the end
    robot->motors->forward(objects[2].path_distance);
  }

  // Signal complete
  delay(200);
  robot->morse->k();
}


/**
 * Technically this is Quiz 14 but its basically a lab
 */
long Quiz14(Marv* robot)
{
  // Measure sensor
  long distance = robot->sensors->frontSonicSensor->measure();
  Serial.println("Distance: " + String(distance) + " cm");
  return distance;
}


// Find and move to the center of a square room
void findAndGoToCenter(Marv* robot)
{
  /**
   * Given the shape of the arena the robot will be placed in, we know that
   * every side will be the exact same length. Thus, we can take 3 measurements: left, forward, right
   * to discern the distance to the left wall, right wall, and wall directly ahead. Using this, we can
   * calculate the distance to the wall behind and therefore devise a location of the robot in the room
   */

   // Measure front, left, and right distances
  long frontWallDist = robot->sensors->frontSonicSensor->measure();
  delay(500);

  robot->stepper->turn(-90);
  delay(500);
  long leftWallDist = robot->sensors->frontSonicSensor->measure();
  delay(500);

  robot->stepper->turn(190);
  delay(500);
  long rightWallDist = robot->sensors->frontSonicSensor->measure();

  delay(500);
  
  // Return stepper to the center
  robot->stepper->turn(-90);
  delay(500);
  
  long sideLen = rightWallDist + leftWallDist;
  long rearWallDist = sideLen - frontWallDist;
  
   // Compute robot's horizontal and vertical distance to the center of the room
   long hDist = (sideLen/2) - min(leftWallDist, rightWallDist);
   long vDist = (sideLen/2) - min(frontWallDist, rearWallDist);

   // Move forward
   if(rearWallDist < frontWallDist)
   {
      robot->motors->forward(vDist);
      delay(500);
      
      if(leftWallDist < rightWallDist)
      {
        robot->motors->turn(90);
      }
      else if (rightWallDist > leftWallDist) {
        robot->motors->turn(-90);
      }
      delay(500);
      robot->motors->forward(hDist);
   }
   // Turn around and move back
   else if(rearWallDist > frontWallDist)
   {
    robot->motors->turn(220);
    delay(500);
    robot->motors->forward(vDist);
    delay(500);
  
    if(leftWallDist < rightWallDist)
      {
        robot->motors->turn(-90);
      }
      else if(leftWallDist > rightWallDist){
        robot->motors->turn(90);
      }
   }
   delay(500);
   robot->motors->forward(hDist);
   delay(100);
}

// Find black line
int findBlackLine(Marv* robot)
{
  uint16_t sensorReadings[8];
  uint16_t calibratedReadings[8];
  uint16_t minReadings[8];
  uint16_t maxReadings[8];

  // Initialize raw and calibrated arrays to all zeros
  //arrays::clearArray(sensorReadings, 8);
  //arrays::clearArray(calibratedReadings, 8);
  
  // Get min and max readings from calibrated line tracker
  robot->sensors->lineTracker->getMinReadings(minReadings);
  robot->sensors->lineTracker->getMaxReadings(maxReadings);

  int lineThreshold = 1000;
  int attempts = 0;
  
   // Read the line sensors then read their calibrated values
   readLineSensor(sensorReadings);
   readCalLineSensor(sensorReadings, calibratedReadings, minReadings, maxReadings, DARK_LINE);
  
  // Move in all directions until the line is found
  while(sensorsActivated(calibratedReadings, 8, lineThreshold) < 1)
  {
    // Turn, move forward, and check for black line
    robot->motors->turn(-1);
    delay(500);
    robot->motors->forward(1);
    delay(500);

    // Read line sensors again
    readLineSensor(sensorReadings);
    readCalLineSensor(sensorReadings, calibratedReadings, minReadings, maxReadings, DARK_LINE);

    // Check again for line
    if(sensorsActivated(calibratedReadings, 8, lineThreshold) < 1)
    {
      // Move back to the original spot and look in next location
      robot->motors->reverse(5);
      delay(500);
    }
    // Line found at new location
    else {
      return 1;
    }

    // Increment number of attempts
    attempts++;

    // Stop if robot can't find the line in a reasonable search area
    if(attempts >= 50)
    {
      return 0;
    }
   }

  return 1;
}

// Front, left, right sweep
void flrSweep(Marv* robot, long* front, long* left, long* right)
{
  // Measure distance in front
  front[0] = robot->sensors->frontSonicSensor->measure();
  delay(50);
  
  // Turn stepper and measure distance to the left
  robot->stepper->turn(-90);
  delay(100);
  left[0] = robot->sensors->frontSonicSensor->measure();

  // Turn stepper and measure distance to the right
  robot->stepper->turn(190);
  delay(100);
  right[0] = robot->sensors->frontSonicSensor->measure();

  // Return servo to center
  robot->stepper->turn(-90);
}

// Find the way out of the maze by following the left wall
void solveMaze(Marv* robot)
{
  int maxWallDist = 240;

  // Setup variables for storing flr sweeps
  long front[1] = {0};
  long left[1] = {0};
  long right[1] = {0};

  // Perform first sweep 
  flrSweep(robot, front, left, right);

  robot->lcd->resetScreen();
  
  // Loop until the left or right measurements are >= maxWallDist
  while(left[0] < maxWallDist && right[0] < maxWallDist)
  {
    // Merge values into a long array
    long vals[3] = {front[0], left[0], right[0]}; 
    int maxIdx = arrays::maxIndex(vals, 3);
    
    // Find the index of the max value
    switch(maxIdx)
    {
      // Front is the farthest away
      case 0:
        robot->lcd->showMessage("Front", -1, 1, 0);
      
        // Move forward half the distance
        robot->motors->forward(vals[maxIdx]/2);
        delay(100);
        break;

      // Left is farthest away
      case 1:
        robot->lcd->showMessage("Left", -1, 1, 0);
      
        // Turn 90deg to the left
        robot->motors->turn(-90);
        delay(750);

        // Move forward half the distance
        robot->motors->forward(vals[maxIdx]/2);
        delay(100);
        break;

      // Right is farthest away
      case 2:
        robot->lcd->showMessage("Right", -1, 1, 0);
        
        // Turn 90deg to the right
        robot->motors->turn(90);
        delay(750);
    
        // Move forward half the distance
        robot->motors->forward(vals[maxIdx]/2);
        delay(100);
        break;
    }

    // Perform the next sweep 
    flrSweep(robot, front, left, right);
  }
}

/**
 * Lab 9 - Part 1
 * 
 * The robot will be placed at a random location in a square with sides that could
 * be from 100cm - 240cm. The robot must find its location in the room and drive towards 
 * the center
 */
 
void lab9(Marv* robot)
{
  // Set line tracker to start calibrating
  robot->sensors->lineTracker->calibrate();

  // Tell robot to start monitoring forward sensors
  robot->motors->monitorFront = true;
  //robot->sensors->monitorBlackLine = true;
  
//  // Find the center of the room and move to it
//  findAndGoToCenter(robot);
//
//  // Start searching for black line
//  if(findBlackLine(robot))
//  {
//    robot->lcd->resetScreen();
//    // Show line follow message
//    robot->lcd->showMessage("Making way", -1, 1, 0);
//    robot->lcd->showMessage("downtown...", -1, 1, 1);
//    
//    // Start following line
//    delay(500);
    
//  }
//  else {
//    robot->lcd->showMessage("Failed to find", -1, 0, 0);
//    robot->lcd->showMessage("black line :(", -1, 0, 1);
//    alarm(500, robot->periphs.buzzer, 5000, 1, RED_LED);
//  }
  
  robot->lcd->resetScreen();
  delay(500);

  robot->sensors->lineTracker->followBlackLine();
  solveMaze(robot);

  // Sound alarm when done
  alarm(500, robot->periphs.buzzer, 350, 3, GREEN_LED);
}



#endif  //  End LABS_H
