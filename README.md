# SYSC 4805 – Autonomous Snowplow

Team Electric Indigo – L2 Group 4  
Carleton University, Fall 2026

## Overview

This repository contains the code for the SYSC 4805 autonomous snowplow project.

The system uses:
- Arduino Due for motor control, IMU, line sensing, state control, battery monitoring and safety.
- Arduino Nano Every for obstacle sensing and watchdog/heartbeat monitoring.

## Repository Structure

```text
Arduino_Due/
└── MainController/

Arduino_Nano_Every/
└── SensorHub/

Unit_Tests/
```

The robot implements:
- strip-clearing navigation
- boundary recovery
- obstacle avoidance
- 7 operating states with RGB indication
- battery and emergency-stop handling
- watchdog and heartbeat fault detection

## Team

- Jacob Lemay
- Nathaniel Baird
- Eliana Schartner
- Betselot Semaw