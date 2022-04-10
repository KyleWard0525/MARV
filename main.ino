/**
 * This file is the main driver for controlling MARV the TI RSLK MAX 
 * Robot
 * 
 * kward
 */
#include <LiquidCrystal_I2C.h>
#include "Servo.h"
#include "Marv.h"
#include "Morse.h"
#include "I2C.h"
#include "IMU.h"
#include "LCD.h"
#include "Labs.h"
#include "Telemetry.h"

Marv* robot;                  //  Main robot driver
Servo servo;                  //  Servo motor object
pins_t periphs;               //  Peripheral pins     

uint32_t maxItrs = 1;
uint8_t itrs = 0;

void testIMU_Session()
{
  test();
  delay(500);
  // Create an IMU session
  IMU_Session session(500);
  session.sampleRateMs = 25;  //  ms betweeen samples

  // Set IMU session
  robot->imu->session = &session;
  
  unsigned long start = millis();
  
  // Poll IMU and print result stored in imu session data
  robot->motors->driveSpeed = 50;

  // Drive a square
  int dist = 30;
  for(int i = 0; i < 4; i++)
  {
    robot->motors->forward(dist);
    delay(500);
    robot->motors->turn(90);
    delay(500);
  }

  // Write measurements over serial
  Serial.println("<IMU_DATA>");
  for(int i = 0; i < session.samples; i++)
  {
      session.data[i].to_string();
  }
  Serial.println("<END>");
}

// For testing new functionality
void test()
{
  double arr[3];
  
  robot->imu->getAccels(arr);
  Serial.print("\nAx = " + String(arr[0]) + "g");
  Serial.print("\tAy = " + String(arr[1]) + "g");
  Serial.print("\tAz = " + String(arr[2]) + "g\n");
  delay(1000);
}

void setup() {  
  // Setup serial output
  Serial.begin(115200);

  // Wait for serial connection to open
  while(!Serial)
  {
    // Do nothing
    ;
  }
  delay(1000);
  
  Serial.print("Serial ready!\n");

  // Setup peripheral pins struct
  periphs.buzzer = 2;             //  P6_0
  periphs.imuSda = 3;             //  P3_2
  periphs.imuClk = 23;            //  P6_1
  periphs.morseLed = 41;          //  P8_5
  periphs.frontTrigPin = 34;      //  P2_3
  periphs.frontEchoPin = 35;      //  P6_7
  periphs.rearTrigPin = 42;       //  P9_0
  periphs.rearEchoPin = 43;       //  P8_4
  periphs.startPin = 63;          //  P6_3
  periphs.pirPin = 46;            //  P6_2
  periphs.servoPin = 67;          //  P9_7


  // Setup servo pin (Must be initialized here, before other setup)
  servo.attach(periphs.servoPin);

  pinMode(periphs.startPin, INPUT_PULLUP);
  pinMode(PUSH2, INPUT_PULLUP);
  
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
  Serial.println("\n\nSetup complete!\n");
}

void loop() {
 
  if(digitalRead(periphs.startPin)==1 || digitalRead(PUSH2)==0)
  {
    Serial.println("Start button pressed!");
    delay(250);
    testIMU_Session();
  }

  
  delay(100); 
}
