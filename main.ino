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
  
  robot = new Marv(buzzerPin, morseLed);

  double arr[3];
  
  robot->imu->getAccels(arr);
  Serial.print("\nAx = " + String(arr[0]) + "g");
  Serial.print("\tAy = " + String(arr[1]) + "g");
  Serial.print("\tAz = " + String(arr[2]) + "g\n");

  robot->imu->getGyros(arr);
  Serial.print("Gx = " + String(arr[0]) + " deg/s");
  Serial.print("\tGy = " + String(arr[1]) + " deg/s");
  Serial.print("\tGz = " + String(arr[2]) + " deg/s\n");

  delay(1000);
}

void loop() {
  
  robot->checkBumpers(); 
}
