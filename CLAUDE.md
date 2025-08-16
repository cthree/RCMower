# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

This project uses arduino-cli for building and uploading. Use the Makefile for standardized commands:

- `make compile` - Compile the project
- `make upload` - Compile and upload to connected board
- `make monitor` - Open serial monitor at 115200 baud
- `make clean` - Clean build artifacts

The Makefile targets Arduino Leonardo by default, but the main code file (`RCMowerRev2Esp.ino`) is designed for ESP32 NodeMCU-32S.

For ESP32 development, you may need to adjust the BOARD_TAG in the Makefile to an appropriate ESP32 board identifier.

## Architecture Overview

This is an ESP32-based RC mower/tank control system with the following key components:

### Hardware Control
- **Motor Control**: Dual DC motors controlled via DS1267B digital potentiometers for speed and MOSFETs for direction
- **RC Input**: Three-channel RC receiver input (throttle, steering, direction) using hardware interrupts
- **Feedback System**: Analog feedback from motor potentiometer wipers for closed-loop control

### Software Architecture
- **Differential Steering**: Tank-style steering with enhanced straight-line compensation using PI control
- **Calibration System**: Multi-stage calibration for both motor speed matching and RC input ranges
- **Safety Features**: Motor enable logic, anti-jarring direction changes, watchdog timer
- **Connectivity**: WiFi with OTA updates, web server for remote monitoring and control
- **Dual-Core Operation**: Main control on core 1, telemetry task on core 0

### Key State Machines
- **CalibrationState**: `CALIB_IDLE`, `CALIB_MOTOR_RUNNING`, `CALIB_RC_SAMPLING`, `CALIB_COMPLETE`
- Motor enable/disable based on throttle centering
- RC pulse width validation and filtering

### Pin Configuration
The system uses specific ESP32 pins for RC inputs (34, 35, 39), motor control (DS1267B on pins 25-27), direction control (32, 33), and safety systems (motor enable pin 14).

## Important Notes

- Motors only enable when throttle is centered within deadband
- The system requires calibration on first use (motor speed matching and RC range)
- WiFi credentials are hardcoded - consider using WiFiManager for production
- Uses ESP32 Preferences instead of EEPROM for persistent storage
- Control loop runs at 20ms intervals with 500ms debug output