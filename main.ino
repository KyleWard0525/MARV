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

int buzzerPin = 2;      //  GPIO pin for the buzzer
int imuSda = 3;         //  Serial data port for the imu
int imuScl = 23;        //  Serial clock for imu
int morseLed = 41;      //  LED pin for blinking messages alongside the audible beeps from the buzzer

Marv* robot;

void setup() {  
  // Setup serial output
  Serial.begin(9600);
  delay(750);

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
  pinMode (buzzerPin, OUTPUT );
  pinMode(imuSda, INPUT);
  pinMode(imuScl, INPUT);
  
  robot = new Marv(buzzerPin, morseLed);

  Serial.print("\nMPU sampling rate from imu: ");
  Serial.print(robot->imu->getSamplingRate());

  double arr[3];
  robot->imu->getAccel(arr);

  Serial.print("\n\nAx = " + String(arr[0]) + "g");
  Serial.print("\tAy = " + String(arr[1]) + "g");
  Serial.print("\tAz = " + String(arr[2]) + "g");
  
}

void loop() {
  // put your main code here, to run repeatedly: 
  robot->checkBumpers(); 
}
