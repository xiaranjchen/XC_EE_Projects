# Self-Righting Ball System
**Electrical Engineering Portfolio Project | Freshman Year**

A control-systems project utilizing an **MPU-6050 IMU** and a **Servo motor** to maintain a level orientation through real-time sensor fusion and feedback loops.

---

## Demo
## 📽️ Project Demo
[![![IMG_20260108_175429391](https://github.com/user-attachments/assets/6439b31b-9fa1-4a9b-a287-6a80082261be)
](https://img.youtube.com/vi/YOUTUBE_ID/0.jpg)](https://drive.google.com/file/d/1xcsY3xpC1quTVqw-IwucDDFRCWHgtK1X/view?usp=sharing)
*Click the image above to watch the system in action.*
---

## Overview
This project implements an embedded system capable of "self-righting" a ball platform. By interpreting raw data from a 6-axis Gyroscope/Accelerometer, the system calculates tilt error and adjusts a servo motor to counteract gravitational pull and external disturbances.

### Technical Highlights:
* **Sensor Fusion:** Implemented a **98/2 Complementary Filter** to combine high-frequency gyroscope data with low-frequency accelerometer stability.
* **Control Theory:** Utilized a **Proportional (P) Controller** to map orientation error to actuator response.
* **Calibration:** Built a 1000-sample startup routine to eliminate static sensor bias (offset).

---

## Hardware Stack
* **Microcontroller:** Arduino (Atmel ATmega328P)
* **IMU Sensor:** MPU-6050 (3-axis Gyro + 3-axis Accelerometer)
* **Actuator:** High-torque Micro Servo
* **Communication:** I2C Protocol (Wire.h)

---

## How it Works
The system solves the two main problems of cheap IMU sensors:
1.  **Gyro Drift:** The gyroscope is accurate but "drifts" over time.
2.  **Accelerometer Noise:** The accelerometer is stable but very "noisy" during quick movements.

**The Math:**
The code uses the following filter logic:
`Angle = 0.98 * (Angle + Gyro_Rate * dt) + 0.02 * (Accel_Angle)`



---

## 💻 Source Code
The following implementation is written in C++/Arduino and uses an Object-Oriented approach for hardware management.

```cpp
/**
Code to help self-righting ball system intepret and make proper adjustments according to inputs for servo to self-correct.
*/
#include <math.h>
#include <Wire.h>   // Communication with the MPU-6050
#include <Servo.h>  // Controlling your servo motor
#include <MPU6050.h>

class Gyroscope {
private:
  // Hardware
  int _servoPin;
  Servo _ballServo;  //Servo
  MPU6050 _mpu;      //Gyroscope and Accelerometer

  // Data Properties
  float _targetAngle;
  float _currentAngle;
  float _gyroOffset[3];
  float _accelOffset[3];

  //Timing Properties
  unsigned long _lastTime;

public:
  //equivalent of initialize_hardware(self)
  Gyroscope(int pin, float target) {
    _servoPin = pin;
    _targetAngle = target;
    _currentAngle = 0.0;
    _lastTime = micros();  //start clock
  }

  void initializeHardware() {
    Serial.println("Initializing Hardware");
    Wire.begin();
    _mpu.initialize();
    _ballServo.attach(_servoPin);
  }

  void calculateOffset() {
    //1000-sample calibration loop
    Serial.println("Calibrating ... keep ball still.");

    //reset Offset lists to 0
    for (int i = 0; i < 3; i++) {
      _gyroOffset[i] = 0;
      _accelOffset[i] = 0;
    }

    int numReadings = 1000;
    int ctr_gyro = 0;
    int ctr_accel = 0;

    while (ctr_gyro < numReadings) {
      ctr_gyro++;

      //fetch raw data
      int16_t gx, gy, gz;
      _mpu.getRotation(&gx, &gy, &gz);

      _gyroOffset[0] += gx;
      _gyroOffset[1] += gy;
      _gyroOffset[2] += gz;

      delay(2);
    }

    while (ctr_accel < numReadings) {
      ctr_accel++;

      //fetch raw data
      int16_t ax, ay, az;
      _mpu.getAcceleration(&ax, &ay, &az);

      _accelOffset[0] += ax;
      _accelOffset[1] += ay;
      _accelOffset[2] += az;

      delay(2);
    }

    for (int idx2 = 0; idx2 < 3; idx2++) {
      float rawAvgGyro = _gyroOffset[idx2] / (float)numReadings;
      float rawAvgAccel = _accelOffset[idx2] / (float)numReadings;

      _gyroOffset[idx2] = rawAvgGyro / 131.0;      // Now in deg/sec
      _accelOffset[idx2] = rawAvgAccel / 16384.0;  // Now in g-force
    }

    _accelOffset[2] = _accelOffset[2] - 1.0;

    Serial.println("Calibration complete!");
  }

  float getCurrentTilt() {
    // Code for the Complementary Filter
    int16_t ax, ay, az;
    _mpu.getAcceleration(&ax, &ay, &az);

    int16_t gx, gy, gz;
    _mpu.getRotation(&gx, &gy, &gz);

    //Scale raw values to human units
    float x_accel_scaled = ((float)ax / 16384.0) - _accelOffset[0];
    float z_accel_scaled = ((float)az / 16384.0) - _accelOffset[2];
    float gyro_rate = ((float)gx / 131.0) - _gyroOffset[0];

    float accel_angle = atan2(x_accel_scaled, z_accel_scaled) * (180 / 3.14159);

    unsigned long currentTime = micros();

    //Calculate dt
    float dt = (currentTime - _lastTime) / 1000000.0;
    _lastTime = currentTime;  //Update times

    //Complemntary filter

    _currentAngle = 0.98 * (_currentAngle + gyro_rate * dt) + 0.02 * accel_angle;
    return _currentAngle;
  }

  float computeServoResponse(float tilt, float gain) {
    float error = _targetAngle - tilt;
    return (error * gain) + 90.0;
  }

  float constrainServoValue(float rawAngle) {
    if (rawAngle < 0) return 0;
    if (rawAngle > 180) return 180;
    return rawAngle;
  }

  void moveServo(float finalAngle) {
    _ballServo.write(finalAngle);  // Physical movement command
  }
};

// Instantiate the ball object (Pin 9, Target 0.0 degrees)
Gyroscope ball(9, 0.0);
// Set the P-Gain for the correction logic
float gain = -1.5;

void setup() {
  Serial.begin(115200);  //initialize Arduino & Computer comms, and speed

  ball.initializeHardware();
  ball.calculateOffset();  //Calibrate while stationary

  Serial.println("Setup Complete!");
}

void loop() {
  // main loop
  float current_tilt = ball.getCurrentTilt();

  float raw_angle = ball.computeServoResponse(current_tilt, gain);

  float final_servo_angle = ball.constrainServoValue(raw_angle);

  ball.moveServo(final_servo_angle);

  Serial.print("Tilt: ");
  Serial.print(current_tilt);
  Serial.print(" | Servo: ");
  Serial.println(final_servo_angle);

  delay(20);
}
