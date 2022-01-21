/**
 * This file is the main driver for controlling MARVIN the TI RSLK MAX 
 * Robot
 * 
 * kward
 */

#include "Marv.h"

int buzzerPin = 2;

Marv robot(buzzerPin);


void setup() {
  
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

  robot.forward(25, 1);
  delay(500);
  robot.turnRight(12, 1);
}

void loop() {
  // put your main code here, to run repeatedly: 
  robot.checkBumpers(); 
}
