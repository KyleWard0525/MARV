/**
 * This file is the main driver for controlling MARVIN the TI RSLK MAX 
 * Robot
 * 
 * kward
 */
#include "Marv.h"
#include "Morse.h"
#include "I2C.h"
#include "IMU.h"
#include "LCD.h"

uint16_t buzzerPin = 2;       //  GPIO pin for the buzzer
uint16_t imuSda = 3;          //  Serial data port for the imu
uint16_t imuScl = 23;         //  Serial clock for imu
uint16_t morseLed = 41;       //  LED pin for blinking messages alongside the audible beeps from the buzzer
uint16_t trigPin = 38;        //  Trigger pin for ultrasonic sensor (signal out)
uint16_t echoPin = 37;        //  Echo pin for ultrasonic signal    (signal in)
uint16_t startPin = 74;       //  Button 2

Marv* robot;

uint32_t maxItrs = 1;
uint8_t itrs = 0;

// Drive square test
void driveSquare(float sideLen)
{
  Serial.println("\nInitial heading: " + String(robot->imu->heading));
  alarm(500, buzzerPin, 500, 3);
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

// For IMU tests
void imuTest()
{
  ;
}

// For testing new functionality
void test()
{
  double arr[3];
  
  robot->imu->getAccels(arr);
  Serial.print("\nAx = " + String(arr[0]) + "g");
  Serial.print("\tAy = " + String(arr[1]) + "g");
  Serial.print("\tAz = " + String(arr[2]) + "g\n");

  double pyr[3];
  robot->imu->getPitchYawRoll(pyr);

  Serial.print("\nPitch = " + String(pyr[0]) + " deg ");
  Serial.print("\tYaw = " + String(pyr[1]) + " deg? ");
  Serial.print("\tRoll = " + String(pyr[2]) + " deg\n");

  Serial.println("\n\nSampling rate: " + String(robot->imu->getSamplingRate()) + "Hz");

  delay(1000);
  
  //robot->displayPitchRoll();
}

void setup() {  
  // Setup serial output
  Serial.begin(9600);

  // Wait for serial connection to open
  while(!Serial)
  {
    // Do nothing
    ;
  }
  delay(1000);
  
  Serial.print("Serial ready!\n");

  pinMode(startPin, INPUT_PULLUP);

  
  // Set bumper pins as inputs
  pinMode(BP_SW_PIN_0, INPUT_PULLUP);
  pinMode(BP_SW_PIN_1, INPUT_PULLUP);
  pinMode(BP_SW_PIN_2, INPUT_PULLUP);
  pinMode(BP_SW_PIN_3, INPUT_PULLUP);
  pinMode(BP_SW_PIN_4, INPUT_PULLUP);
  pinMode(BP_SW_PIN_5, INPUT_PULLUP);

  // Set RGB LEDs as outputs
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(morseLed, OUTPUT);
  
  // Set buzzer pin as output
  pinMode(buzzerPin, OUTPUT);

  // Setup ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);


  robot = new Marv(buzzerPin, morseLed, trigPin, echoPin);
  
  robot->morse->i(); 

  delay(2000);
  Serial.println("\n\nEnd setup()\n");
}

void loop() {
//  if(itrs >= maxItrs)
//  {
//    Serial.println("\nEnd of program\n");
//    exit(0);
//  }

  if(digitalRead(startPin) == 0)
  {
    driveSquare(30.5*3);
  }

  robot->checkBumpers();

  // Print accels and gyro readings
  //Serial.println("Ax = " + String(imuVals[0]) + "\tAy = " + String(imuVals[1]) + "\tAz = " + String(imuVals[2])
                  //+ "\nGx = " + String(imuVals[3]) + "\tGy = " + String(imuVals[4]) + "\tGz = " + String(imuVals[5]) + "\n");
  
  // Print IMU derivatives
  //Serial.println("\nHeading = " + String(robot->imu->heading) + "deg\tVelocity = " + String(robot->imu->velocity) + " cm/s\tDistance traveled: " + String(robot->imu->distance) + " cm");
  
  delay(200);

  itrs++;  
}
