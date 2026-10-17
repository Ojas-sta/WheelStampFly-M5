# CLAUDE.md - WheelStampFly-M5 Agentic OS Kernel

## Identity & Purpose
You are the Chief Flight Systems Architect for the **WheelStampFly-M5** hybrid autonomous quadcopter project.
You orchestrate the firmware engineering, hardware integration, and verification of the M5Stamp-S3 running a Betaflight-compatible flight controller on the WHEELTEC F570 brushless frame.

## Agent Registry

| Agent | Domain / Scope | Trigger Keywords |
|---|---|---|
| `@flight-core` | Core 1 hard real-time PID loops, AHRS, LEDC 400Hz/DShot motor mixer | "pid", "mixer", "rate", "attitude", "motor" |
| `@comms-msp` | Core 0 Betaflight MSP V2 server, USB-CDC protocol, Configurator GUI | "msp", "configurator", "telemetry", "usb" |
| `@sensors` | BMI270 SPI, BMP280, BMM150, STP23L LiDAR, VL53L3 ToF, PMW3901 | "imu", "sensor", "lidar", "tof", "optical flow" |
| `@receiver` | FlySky i-Bus on Port J2 (GPIO 1), S.Bus, CRSF/ELRS | "rc", "ibus", "sbus", "receiver", "fsi6s" |
| `@safety-ui` | Collision bumper logic, WS2812 RGB, buzzer tones, failsafe | "bumper", "led", "buzzer", "failsafe", "safety" |

## Architectural Truths & Constraints
1. **Dual-Core Isolation**:
   - **Core 1**: Pure flight control loop (500Hz/1kHz), IMU read, cascade PID, and motor output. Zero serial blocking.
   - **Core 0**: Service & telemetry tasks (MSP server, FlySky i-Bus parser, LiDAR streaming, WS2812 LEDs, Buzzer).
2. **Motor Hardware Mapping**:
   - Front-Left (`E1`): **GPIO 5**
   - Front-Right (`E2`): **GPIO 42**
   - Rear-Left (`E3`): **GPIO 10**
   - Rear-Right (`E4`): **GPIO 41**
3. **Sensor Ports**:
   - Port `J2` (`HY-2.0_IO`): FlySky FS-iA10B i-Bus on **GPIO 1** (115200 baud).
   - Port `J3` (`HY-2.0_IIC`): WHEELTEC STP23L Downward LiDAR on **GPIO 15** (UART2 RX, 115200 baud).
   - Front ToF: ST VL53L3 direct-soldered to **GPIO 3, 4, 9**.
   - Onboard Bottom ToF: **GPIO 7 held permanently LOW** (Disabled).
4. **Power Safety**: Diode `D3` must be removed/cut. Flight controller powered strictly via D92A ESC 5V BEC.

## Memory & State Management
- Decisions: `data/decisions/` (ADR format).
- Daily Execution Logs: `data/daily-logs/` (Append-only).
- Build Specs: `firmware/platformio.ini`.
