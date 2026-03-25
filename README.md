# Autonomous Line-Following Robot with Gyroscope Integration

This project implements an **autonomous line-following robot** using the **ESP32 microcontroller**, integrated with an **MPU6050 gyroscope** to perform **precise 90° junction turns**. The system uses **ESP-IDF** and **FreeRTOS** for multitasking and real-time control.

---

## Table of Contents

- [Features](#features)  
- [Hardware](#hardware)  
- [Software](#software)  
- [System Architecture](#system-architecture)  
- [Code Structure](#code-structure)  
- [Control Logic](#control-logic)  
- [Calibration](#calibration)
- [Execution Flow](#execution-flow)  
- [Future Improvements](#future-improvements)  

---

## Features

- Real-time **junction detection** using 8 IR sensors  
- **PID-based line-following** with weighted sensor array  
- **Gyroscope-assisted yaw estimation** for accurate 90° turns  
- **State-based control** for switching between line-following and junction handling  
- Modular code structure for sensor, motor, and task management  

---

## Hardware

- **Microcontroller:** ESP32  
- **Gyroscope/Accelerometer:** MPU6050
- **Motors:** N20 Micro Metal Gear Motors (6V, 200 RPM) with integrated encoders (×2)    
- **Sensors:** 8 IR reflectance sensors  
- **Communication:** UART (optional Bluetooth handled by team)  

---

## Software

- **Framework:** ESP-IDF  
- **RTOS:** FreeRTOS  
- **Programming Language:** C  
- **Development Tools:** VSCode / ESP-IDF extension, PlatformIO optional  

---

## System Architecture

The robot software is organized as **multiple FreeRTOS tasks** running in parallel:

| Task | Core | Responsibility |
|------|------|----------------|
| `GyroTask` | 0 | Reads MPU6050 gyro Z-axis, calculates yaw, and updates shared state |
| `SensorTask` | 1 | Reads 8 IR sensors, normalizes values, executes line-following and junction detection logic |
| `UART_Task` | 0 | Handles UART commands (RUN, STOP, CAL) for debugging and control |
| Encoder ISRs | N/A | Increment wheel encoder counts(for future purposes) |

The system uses **mutex locks** (`portMUX_TYPE`) to protect shared variables like `yaw` across tasks.

---

## Code Structure

- `main.c` – Entry point (`app_main`) and FreeRTOS task creation  
- `i2c_driver.c` – Low-level I2C functions to communicate with MPU6050 (`i2c_read_bytes`, `i2c_write_byte`)  
- `gyro.c` – Gyroscope initialization, reading, and calibration  
- `sensors.c` – IR sensor reading, normalization, and pattern detection  
- `motors.c` – Motor driver functions, PWM control, and turn handling  
- `control.c` – PID control logic, junction state management  
- `uart.c` – UART command parser and status reporting  

---

## Control Logic

### 1. Line Following (PID)

- Weighted sum of 8 sensors determines **line position error**:

```c
float err = line_position();
motor_set(BASE_PWM + Kp * err, BASE_PWM - Kp * err);
```

- PID controller uses **only proportional term** for real-time adjustments.

---

### 2. Junction Detection

Detects **T-junctions, left, and right junctions** using IR sensor patterns:

```c
curr_pattern = sensor_pattern();

if (curr_pattern == 0b11111111)
    t_cnt++;
else if (curr_pattern == LEFT_PATTERN)
    left_cnt++;
else if (curr_pattern == RIGHT_PATTERN)
    right_cnt++;
```

- False patterns (curves or circles) are rejected using **white-space validation**.

---

### 3. Gyro-Assisted Turns

After detecting a junction, target yaw is calculated:

```c
turn_target_yaw = fmodf(curr_yaw + 85.0f, 360.0f); // Right turn
```

- Uses real-time yaw feedback from MPU6050  
- Ensures accurate 90° rotation  

---

### 4. State Management

Flags used for smooth mode switching:

| Flag | Meaning |
|------|--------|
| robot_run | PID line-following active |
| junction_detected | Junction in progress |
| pid_hold | Temporarily pauses PID during turns |

---

##  Calibration

### Sensor Calibration

```c
do_calibration = true;
parse_line("BLACK");   // black reference
parse_line("WHITE");   // white reference
```

---

### Gyroscope Calibration

```c
gyro_z_bias = sum / CALIB_SAMPLES;
```

- Reduces drift and improves accuracy

---

## Execution Flow

- Sensor data acquisition and normalization  
- Real-time line position estimation using weighted averaging  
- PID control loop for motor correction  
- Junction detection using binary pattern recognition  
- Gyro-based yaw tracking for closed-loop turn control  
- State-based switching between navigation modes  

## Future Improvements

- Implement full PID tuning(add D term)  
- Implement a two-pass maze-solving algorithm where the robot explores and maps the maze in the first run, and computes an optimized shortest path for efficient navigation in the second run.

---
