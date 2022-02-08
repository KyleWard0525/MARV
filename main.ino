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
uint16_t trigPin = 38;         //  Trigger pin for ultrasonic sensor (signal out)
uint16_t echoPin = 37;         //  Echo pin for ultrasonic signal    (signal in)

Marv* robot;

void setup() {  
  // Setup serial output
  Serial.begin(9600);

  // Wait for serial connection to open
  while(!Serial)
  {
    // Do nothing
    ;
  }

  Serial.println("Serial ready!");

  
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

  double arr[3];
  
  robot->imu->getAccels(arr);
  Serial.print("\nAx = " + String(arr[0]) + "g");
  Serial.print("\tAy = " + String(arr[1]) + "g");
  Serial.print("\tAz = " + String(arr[2]) + "g\n");

  double pr_arr[2];
  robot->imu->getPitchRoll(pr_arr);

  Serial.print("\n\nPitch = " + String(pr_arr[0]) + " deg ");
  Serial.print("\tRoll = " + String(pr_arr[1]) + " deg");;

  delay(2000);

  double distance = robot->sonicSensor->measure();
  Serial.println("\nDistance measured: " + String(distance));
}

void loop() {
  
  //robot->sonicSensor->measure(); 
  robot->checkBumpers();
}
