
/**
 * This file is the main driver for controlling MARV the TI RSLK MAX 
 * Robot
 * 
 * kward
 */
#include <LiquidCrystal_I2C.h>
#include "Servo.h"
#include "Marv.h"
#include "LCD.h"
#include "Labs.h"
#include "Tests.h"
#include "StepMotor.h"

Marv* robot;                  //  Main robot driver
Servo servo;                  //  Servo motor object
StepMotor* stepper;           //  Stepper motor
pins_t periphs;               //  Peripheral pins     

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
  //periphs.rearTrigPin = 42;       //  P9_0
  //periphs.rearEchoPin = 43;       //  P8_4
  periphs.startPin = 63;          //  P6_3
  periphs.pirPin = 46;            //  P6_2
  periphs.servoPin = 67;          //  P9_7
  periphs.lineSensor_0 = 65;      //  P7_0 (NOTE: All line sensor pins are hardwired on the board)
  periphs.lineSensor_1 = 48;      //  P7_1
  periphs.lineSensor_2 = 64;      //  P7_2
  periphs.lineSensor_3 = 47;      //  P7_3
  periphs.lineSensor_4 = 52;      //  P7_4
  periphs.lineSensor_5 = 68;      //  P7_5
  periphs.lineSensor_6 = 53;      //  P7_6
  periphs.lineSensor_7 = 69;      //  P7_7
  periphs.lineSensor_Even = 61;   //  P5_3
  periphs.lineSensor_Odd = 45;    //  P9_2 
  periphs.bumper_0 = BP_SW_PIN_0; //  Bumper switch 0 (FAR RIGHT BUMPER)
  periphs.bumper_1 = BP_SW_PIN_1; //  Bumper switch 1
  periphs.bumper_2 = BP_SW_PIN_2; //  Bumper switch 2 
  periphs.bumper_3 = BP_SW_PIN_3; //  Bumper switch 3
  periphs.bumper_4 = BP_SW_PIN_4; //  Bumper switch 4
  periphs.bumper_5 = BP_SW_PIN_5; //  Bumper switch 5 (FAR LEFT BUMPER)
  periphs.rLed = RED_LED;         //  Onboard Red LED
  periphs.gLed = GREEN_LED;       //  Onboard Green LED
  periphs.bLed = BLUE_LED;        //  Onboard Blue LED
  periphs.stepIn_1 = 42;          //  P9_0
  periphs.stepIn_2 = 43;          //  P8_4
  periphs.stepIn_3 = 44;          //  P8_2
  periphs.stepIn_4 = 60;          //  P8_3

  // Setup servo pin (Must be initialized here, before other setup)
  servo.attach(periphs.servoPin);
  
  pinMode(periphs.startPin, INPUT_PULLUP);
  pinMode(PUSH2, INPUT_PULLUP);
  
  // Set bumper pins as inputs
  pinMode(periphs.bumper_0, INPUT_PULLUP);
  pinMode(periphs.bumper_1, INPUT_PULLUP);
  pinMode(periphs.bumper_2, INPUT_PULLUP);
  pinMode(periphs.bumper_3, INPUT_PULLUP);
  pinMode(periphs.bumper_4, INPUT_PULLUP);
  pinMode(periphs.bumper_5, INPUT_PULLUP);

  // Set RGB LEDs as outputs
  pinMode(periphs.rLed, OUTPUT);
  pinMode(periphs.gLed, OUTPUT);
  pinMode(periphs.bLed, OUTPUT);
  pinMode(periphs.morseLed, OUTPUT);
  
  // Set buzzer pin as output
  pinMode(periphs.buzzer, OUTPUT);

  // Setup ultrasonic sensor pins
  pinMode(periphs.frontTrigPin, OUTPUT);
  pinMode(periphs.frontEchoPin, INPUT);
  //pinMode(periphs.rearTrigPin, OUTPUT);
  //pinMode(periphs.rearEchoPin, INPUT);

  // Setup PIR Sensor pins
  pinMode(periphs.pirPin, INPUT);
  
  robot = new Marv(periphs, &lcdI2C_module);
  Serial.println("\n\nSetup complete!\n");
  
}

void loop(){
 
  if(digitalRead(periphs.startPin)==1 || digitalRead(PUSH2)==0)
  {
    delay(500);

    //testFollowBlackLine(robot);
    //solveMaze(robot);
    lab9(robot);
  }

  robot->sensors->monitorForwardSensors();
  
  delay(100); 
}
