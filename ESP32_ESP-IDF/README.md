# ESP32 MPU6050 High-Stability FRC Gyro

A high-precision gyro implementation for ESP32 and MPU6050, optimized for FRC (First Robotics Competition) robots. This system uses advanced sensor fusion and drift rejection algorithms to achieve near NavX-grade stability (approx. 0.005°/s drift) on a 6-DOF sensor.

## Features

1.  **DMP (Digital Motion Processing):** Leverages the MPU6050's internal hardware for initial 6-axis sensor fusion.
2.  **10-Second OLS Calibration:** Performs Ordinary Least Squares linear regression at startup for highly accurate initial bias estimation.
3.  **ZUPT (Zero-Velocity Update):** Intelligent detector (accelerometer variance + gyro magnitude) identifies stationary periods.
4.  **Yaw Freeze State Machine:** Hard-freezes the yaw output during ZUPT periods to ensure absolutely zero stationary drift.
5.  **Online Kalman Filter:** Continuously tracks gyro bias random-walk in the background using a 1D Kalman filter.
6.  **Temperature Compensation:** Real-time online regression of bias vs. temperature (`∂bias/∂T`) to handle thermal drift.
7.  **EMI Glitch Rejection:** Detects and discards non-physical yaw jumps caused by motor controller electromagnetic interference (EMI).
8.  **Fused Output:** Seamlessly resynchronizes accumulated corrections when motion resumes.

## Requirements

-   ESP32 (ESP-IDF v5.2+)
-   MPU6050 Accelerometer/Gyroscope
-   I2C Connection (Default: SDA=21, SCL=22)

## Installation

1.  Clone this repository into your ESP-IDF workspace.
2.  Set up your ESP-IDF environment:
    ```bash
    . $HOME/esp-idf/export.sh
    ```
3.  Build and flash:
    ```bash
    idf.py build
    idf.py -p [PORT] flash monitor
    ```

## Usage

Ensure the robot is completely stationary for the first 13 seconds (3s warmup + 10s OLS calibration). The console output will log the initial bias and temperature coefficient.

The Public API provides:
- `gyro_getYawDeg()`: Mutex-protected thread-safe yaw reading.
- `gyro_resetYaw()`: Resets current yaw to zero (offset management).

## Performance Note

While this software stack significantly improves MPU6050 stability, it is a 6-DOF system (no magnetometer). Absolute heading reliability over long periods (e.g., full matches) will be ~2-4° depending on total rotation count. For 9-DOF performance, consider hardware with an integrated magnetometer.
