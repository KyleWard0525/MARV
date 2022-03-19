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

Marv* robot;                  //  Main robot driver
pins_t periphs;               //  Peripheral pins

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

  // Setup peripheral pins struct
  periphs.buzzer = 2;
  periphs.imuSda = 3;
  periphs.imuClk = 23;
  periphs.morseLed = 41;
  periphs.frontTrigPin = 32;
  periphs.frontEchoPin = 31;
  periphs.rearTrigPin = 42;
  periphs.rearEchoPin = 43;
  periphs.startPin = 17;
  periphs.pirPin = 46;

  pinMode(periphs.startPin, INPUT_PULLUP);
  
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
  pinMode(periphs.morseLed, OUTPUT);
  
  // Set buzzer pin as output
  pinMode(periphs.buzzer, OUTPUT);

  // Setup ultrasonic sensor pins
  pinMode(periphs.frontTrigPin, OUTPUT);
  pinMode(periphs.frontEchoPin, INPUT);
  pinMode(periphs.rearTrigPin, OUTPUT);
  pinMode(periphs.rearEchoPin, INPUT);

  // Setup PIR Sensor pins
  pinMode(periphs.pirPin, INPUT);

  robot = new Marv(periphs, &lcdI2C_module);

  Serial.println("\n\nEnd setup()\n");
}

void loop() {
//  if(itrs >= maxItrs)
//  {
//    Serial.println("\nEnd of program\n");
//    exit(0);
//  }

    // Check PIR Sensor
    int pirState = digitalRead(periphs.pirPin);
    if(pirState == HIGH)
    {
      Serial.println("PIR Triggered!");
      digitalWrite(RED_LED,HIGH);
    }

  // Measure distance to nearest object in front and rear
//  long front_dist = robot->frontSonicSensor->measure();
//  robot->lcd->showMessage("Front: " + String(front_dist) + "cm", -1, 0, 0);
//  long rear_dist = robot->rearSonicSensor->measure();
//  robot->lcd->showM essage("Rear: " + String(rear_dist) + "cm", -1, 0, 1);
  
  if(digitalRead(periphs.startPin) == 1)
  {
    Serial.println("Start button pressed!");
    alarm(300, periphs.buzzer, 500, 1, GREEN_LED);
  }
  
  delay(500);

  itrs++;  
}
