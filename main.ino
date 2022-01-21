/**
 * This file is the main driver for controlling MARVIN the TI RSLK MAX 
 * Robot
 * 
 * kward
 */

#include "Marv.h"
#include "Morse.h"
#include <Wire.h>
#include <math.h>

int buzzerPin = 2;      //  GPIO pin for the buzzer
int imuSda = 3;         //  Serial data port for the imu
int imuScl = 23;        //  Serial clock for imu


Marv robot(buzzerPin);
Morse morse(buzzerPin);

void setup() {
  Wire.begin();
  
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

  // Set buzzer pin as output
  pinMode(buzzerPin, OUTPUT);
  pinMode(imuSda, INPUT);
  pinMode(imuScl, INPUT);

  // Wake up MPU
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); // Sleep register
  Wire.write(1);
  Wire.endTransmission();

  Serial.print("\nMPU Identifier (should be 104): ");
  Serial.print(readByte(0x68, 0x75));


  // Read MPU's sleep bit 
  Serial.print("\nMPU sleep bit value: ");

  // 0x68 = MPU I2C address, 0x6b = MPU power control register, 6 = position of sleep bit in the byte
  Serial.print(readBit(0x68, 0x6b, 6));

  uint8_t res = 0;
  
  for(int i = 6; i >=1 ; i--)
  {
      res += readBit(0x68, 0x75, i);
  }

  Serial.print("\nMPU identifier (read bit-by-bit): ");
  Serial.print(res);

  Serial.print("\n\n");
  byte val = readByte(0x68, 0x3b);
  Serial.print(val);

  res = 0;

  for(int i = 0; i < 8; i++)
  {
    res += readBit(0x68, 0x3b, i);
  }

  Serial.print("\n");
  Serial.print(res);

//  robot.forward(25, 1);
//  delay(500);
//  robot.turnRight(12, 1);
}

void loop() {
  // put your main code here, to run repeatedly: 
  robot.checkBumpers(); 
}
