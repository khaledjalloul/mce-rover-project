# MCE Project

This project is part of the MCE System Design course, and aims to detect small fires using a camera and a flame sensor then attempt to put it out.

## Hardware Components

The components used in the project:

- Raspberry Pi 3B+
- USB Camera
- Four TT130 DC motors
- L293D Motor Driver
- Two LM393 encoders
- LM393 flame sensor
- DC30A-1230 12V Water pump

## Software

The software consists of two python scripts:

### motor_controller.py

This script is responsible for defining the motor and ecnoder pins, as well as initiate the PWM channels. A move() function is defined to move the robot forward or backward, or rotate it left or right. Every 0.5s, the incremented counters of the encoders are used to regulate the speed of the motors to 30 cm/s using a simple PID controller.

### main_control.py

This script is the robot's main control. Every loop, the flame sensor digital value is checked. If there is no nearby fire, the camera captures frames in front of the robot, then based on the HSV color values, detects a flame. Based on the location of the flame – right, left, or center – the robot will rotate and move towards the flame.

Once it reaches its vicinity, the flame sensor will activate, thus actuating the pump. The pump will start spraying water and remain at it until the flame is no longer detected, then the robot will go back to searching using the camera.

## Demo

A lighter and a printed image were used to simulate a small fire.

![Demo](demo.gif)