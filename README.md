# MARV (Mildly Autonomous Robotic Vehicle)
MARV is being designed and built as a way for me to gain an introdcution to robotics and hopefully make a little robot friend.

## Hardware
As of v0.0.1 (1/21/2022), MARV uses the following hardware:
- TI RSLK-MAX Robotics kit
- TI MSP432P401R LaunchPad Development board (Arduino)
- MPU-6050 Inertial Measurement Unit
- 6 Forward-Facing Bump Sensors
- 2 DRV8838 Motor Drivers for the 2 DC Motors
- Active Buzzer
- Dual-Nonfunctional Googly Optical Recognition Sensors

## Software
All of the software is being written in C/C++ using the Energia IDE

## Features
![MARV v0.0.1](images/MARV0.0.1.jpg)
***MARV v0.0.1 shown above***

As of version 0.0.1, MARV features:
- Forward collision detection
- Collision alert (via onboard LED and active buzzer)
- Morse code communication through LED blinks and buzzer beeps

## Upcoming Updates:
- Collision avoidance
- Follow mode (robot follows the user around)
- Use measurements from IMU to determine orientation in order to get out of a bad spot
- Add oscilating ultrasonic sensor assembly using stepper motor and 3 ultrasonic sensors in a triangle configuration in attempt to create a 3D map of the robot's current environment using ultrasound