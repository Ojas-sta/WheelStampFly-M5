# ADR-001: Standalone Betaflight-Compatible Firmware Architecture

**Date**: 2026-10-17  
**Status**: Accepted  
**Deciders**: User, Architect, Council  

## Context
We are converting the WHEELTEC F570 brushless quadcopter (1104 motors, D92A 4-in-1 ESC, 3S battery) using the M5Stamp-S3 and StampFly sensor board as the flight computer. The user requires full compatibility with the official Betaflight Configurator desktop GUI, support for the FlySky FS-i6S transmitter over i-Bus, an active forward collision avoidance bumper, and hybrid ground LiDAR altitude hold, with zero motor jitter or cycle-stealing from cosmetics.

## Decisions

1. **Codebase Foundation**:
   - Extract the self-contained Betaflight MSP server, AHRS, i-Bus parser, and hardware drivers from `madflight` into a lightweight, standalone bespoke firmware repository inside `WheelStampFly-M5/firmware`.
2. **Dual-Core Task Separation**:
   - **Core 1 (Real-Time Flight Core)**: Dedicated exclusively to high-rate IMU sampling (BMI270 SPI @ 1kHz), Mahony/Madgwick fusion, dual-loop cascade PID, and 400Hz LEDC / DShot motor outputs.
   - **Core 0 (Service Core)**: Handles USB-CDC MSP protocol communications with the Betaflight Configurator, FlySky i-Bus parsing, STP23L LiDAR serial parsing, front VL53L3 ToF ranging, WS2812 RGB LED animations, and buzzer tones.
3. **Motor Output**:
   - Default: Hardware LEDC 400Hz Servo PWM ($1000\mu\text{s} - 2000\mu\text{s}$) on GPIO 5, 42, 10, 41.
   - Configurable: Toggle switch via Betaflight CLI to enable DShot300 via ESP32 RMT peripheral.
4. **RC Receiver Protocol**:
   - Primary: FlySky i-Bus on Port `J2` (`GPIO 1`) at 115,200 baud.
   - Configurator-switchable to S.Bus (with ESP32-S3 software inversion) and CRSF/ELRS.
   - ESP-NOW Atom Joystick disabled to eliminate RF contention and minimize RAM overhead.
5. **Forward Collision Bumper**:
   - Front VL53L3 ToF direct-soldered to GPIO 3, 4, 9.
   - Proportional pitch clamping below $1.0\text{m}$ and active reverse soft-braking below $0.5\text{m}$.
   - Toggled via an AUX switch in the Betaflight Modes tab.
6. **Altitude & Position Assistance**:
   - Downward WHEELTEC STP23L 1D LiDAR connected to Port `J3` (`GPIO 15`, UART2 RX).
   - Hybrid laser ground tracking ($0 - 4\text{m}$) with seamless BMP280 baro fallback.
   - Assisted Auto-Takeoff & Hover mode on AUX switch (climbs to 1.0m and locks position using PMW3901 optical flow).

## Consequences
- **Positive**: Hard real-time flight stability on Core 1 with zero timing jitter from USB/serial or LED updates; complete plug-and-play compatibility with Betaflight Configurator GUI; fully functional collision avoidance and precision indoor hover.
- **Negative / Mitigations**: Requires careful thread-safe message passing (FreeRTOS queues / atomic variables) between Core 0 and Core 1.
