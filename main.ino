/**
 * This file is the main driver for controlling MARV the TI RSLK MAX 
 * Robot
 * 
 * kward
 */
#include <LiquidCrystal_I2C.h>
#include "Marv.h"
#include "Morse.h"
#include "I2C.h"
#include "IMU.h"
#include "LCD.h"
#include "Labs.h"

uint16_t buzzerPin = 2;       //  GPIO pin for the buzzer
uint16_t imuSda = 3;          //  Serial data port for the imu
uint16_t imuScl = 23;         //  Serial clock for imu
uint16_t morseLed = 41;       //  LED pin for blinking messages alongside the audible beeps from the buzzer
uint16_t trigPin = 32;        //  Trigger pin for ultrasonic sensor (signal out)
uint16_t echoPin = 31;        //  Echo pin for ultrasonic signal    (signal in)
uint16_t startPin = 74;       //  Button 2
uint16_t pirPin = 50;

Marv* robot;

uint32_t maxItrs = 1;
uint8_t itrs = 0;

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

  robot->motors->forward(5);
  delay(500);
  robot->motors->turn(90);
  
  delay(1000);
  
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

  // Setup PIR Sensor pins
  pinMode(pirPin, INPUT);

  robot = new Marv(buzzerPin, morseLed, trigPin, echoPin, pirPin, &lcdI2C_module);

  Serial.println("\n\nEnd setup()\n");
}

void loop() {
//  if(itrs >= maxItrs)
//  {
//    Serial.println("\nEnd of program\n");
//    exit(0);
//  }

  // Measure distance to nearest object
  long dist = robot->frontSonicSensor->measure();
  robot->lcd->showMessage("Distance: " + String(dist) + "cm", -1, 0, 0);

  if(digitalRead(startPin) == 0)
  {
    lab5Demo(robot);
  }

  //robot->checkBumpers();
  //robot->monitorForwardSensors();
  
  delay(200);

  itrs++;  
}
