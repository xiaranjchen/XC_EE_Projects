# XC_EE_Projects
Documentation and source code for avionics sensors and PCB designs created during freshman year winter.

# Self-Righting Ball System
An Electrical Engineering project using an MPU-6050 and Servo to maintain orientation.

### Project Overview
This project uses a **Complementary Filter** to combine data from a gyroscope and an accelerometer. This allows the system to ignore short-term noise and long-term drift, ensuring the ball stays level.

### Hardware Used
* **Microcontroller:** Arduino Uno/Nano
* **Sensor:** MPU-6050 (Gyroscope + Accelerometer)
* **Actuator:** SG90 Servo Motor

### How it Works
1. **Calibration:** On startup, the system takes 1000 samples to find the 'zero' offset.
2. **Filtering:** Uses a 98/2 ratio complementary filter for stability.
3. **Control:** Implements a P-controller (Proportional) to map tilt error to servo movement.
