# SYSC 4805 Autonomous Snowplow

This repository contains the software developed for the **SYSC 4805 Computer Systems Design Lab – Autonomous Snowplow Project**.

## Project Overview

The goal of the project is to design and implement an autonomous robot capable of navigating a bounded arena, detecting and avoiding obstacles, remaining inside the black-tape boundary, and clearing simulated snow cubes from the arena.

The system uses two microcontrollers:

- **Arduino Due** – primary motion and navigation controller
- **Arduino Nano Every** – obstacle-sensing subsystem

The software is being developed incrementally, with independent component handlers created and tested before system integration.

## Current Software

### Arduino Due

Current Due-side development includes:

- Motor control
- Motor speed control
- Emergency motor stop functionality
- MinIMU-9 v5 / LSM6 initialization
- Gyroscope calibration
- IMU-based heading measurements
- Left and right turning routines
- Motor + IMU integration
- I2C bus recovery
- Timed safety fallbacks for turning

### Arduino Nano Every

Current Nano-side development includes:

- Two digital IR obstacle sensors
- Ultrasonic distance sensor
- VL53L1X range-finder sensor
- Non-blocking ultrasonic measurements
- Sensor timeout handling
- Sensor-event notification to the Arduino Due
- Digital interrupt-style communication between the Nano and Due

## Repository Structure

```text
SYSC4805_L2G5/
├── Arduino_Due/
│   ├── IMU_Calibration/
│   └── IMU_Motor_Turn_Baseline/
│
├── Arduino_Nano_Every/
│   └── SensorHub/
│
└── README.md
```

Additional component handlers will be added as development continues.

## Due–Nano Communication

The Arduino Nano Every monitors the obstacle-detection sensors and notifies the Arduino Due when a relevant sensor event occurs.

Dedicated digital notification lines are used for:

- IR sensor 1
- IR sensor 2
- Ultrasonic sensor
- VL53L1X range finder

The Arduino Due uses these sensor notifications as inputs to the robot's higher-level navigation and state-control logic.

## Component Development

The project follows an independent-component-first development approach. Individual sensor and actuator handlers are developed and tested before being integrated into the complete robot.

Current or planned component handlers include:

- Motor control
- IMU / gyroscope
- Line and boundary sensing
- IR obstacle sensing
- Ultrasonic obstacle sensing
- VL53L1X range sensing
- RGB state indication
- Battery monitoring
- Watchdog / fault recovery
- Emergency stop
- Navigation and state-machine control

## Testing

Component testing is performed using measurable success criteria defined in the project proposal and progress report.

Testing includes:

- Motor speed and turning accuracy
- IMU heading accuracy
- Boundary detection
- Obstacle detection and response
- Sensor communication reliability
- Battery monitoring
- RGB state indication
- Watchdog fault recovery
- Emergency-stop response
- Incremental system integration

## Development Process

Independent component development is performed before system integration.

Integration is carried out incrementally, beginning with:

1. Motors + IMU
2. Boundary sensing
3. Obstacle sensors
4. Navigation and snow-clearing behaviour
5. Full sensor, safety, and state-machine integration

## Team

**Team Electric Indigo**

- Jacob Lemay
- Nathaniel Baird
- Eliana Schartner
- Betselot Semaw

Carleton University  
Department of Systems and Computer Engineering  
SYSC 4805 – Computer Systems Design Lab  
Fall 2026