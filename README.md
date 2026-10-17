# WHEELTEC F570 × M5StampFly Hybrid Brushless Drone

[![Platform](https://img.shields.io/badge/Platform-ESP32--S3-orange.svg)](https://www.espressif.com/)
[![Framework](https://img.shields.io/badge/Framework-Arduino%20%2F%20ESP--IDF-blue.svg)](https://platformio.org/)
[![Flight%20Control](https://img.shields.io/badge/Ecosystem-Betaflight%20Compatible-brightgreen.svg)](https://betaflight.com/)
[![Motors](https://img.shields.io/badge/Motors-1104%20Brushless%20(4300KV)-red.svg)]()
[![Battery](https://img.shields.io/badge/Battery-3S%20LiPo%20(11.1--12.6V)-yellow.svg)]()

A high-performance autonomous hybrid drone combining the **WHEELTEC F570** carbon chassis, 1104 brushless motors, and D92A 4-in-1 ESC with the **M5Stack Stamp-S3** and **StampFly sensor deck** as the core flight computer.

---

## 🚁 Project Overview

This project replaces the stock STM32 C18C flight controller on the WHEELTEC F570 with the **M5Stamp-S3 + StampFly multi-sensor array**, creating a lightweight, high-thrust aerial robotics research platform with:
* **High-Power Propulsion**: 1104 brushless motors (4300KV) on 3S (11.1V–12.6V) with the D92A 4-in-1 ESC.
* **Full Multi-Sensor Array**:
  * **IMU**: Bosch BMI270 (6-axis Accel/Gyro via SPI)
  * **Barometer**: Bosch BMP280 (I2C)
  * **Magnetometer**: Bosch BMM150 (3-axis Compass via I2C)
  * **Optical Flow**: PixArt PMW3901 (SPI)
  * **Downward Altitude**: WHEELTEC STP23L 1D Laser LiDAR (via UART on Port `J3`)
  * **Forward Collision Bumper**: ST VL53L3CX Time-of-Flight sensor (direct-soldered I2C)
* **RC Control**: FlySky FS-i6S with FS-iA10B receiver over **i-Bus** on Port `J2` (with runtime switching for S.Bus and ELRS / CRSF).
* **Betaflight Compatibility**: Official Betaflight Configurator desktop GUI support for live 3D preview, rate tuning, PID adjustments, and motor bench tests.

---

## ⚡ Direct Solder Wiring Diagram (D92A ESC to Stamp-S3)

```
       WHEELTEC D92A 4-in-1 ESC BOARD                    M5STAMP-S3 MODULE
     ┌────────────────────────────────┐            ┌────────────────────────────┐
     │                                │            │       [ USB-C Port ]       │
     │  [3S LiPo Power Pads]          │            │                            │
     │   (+) RED   (11.1V - 12.6V)    │            │  Header P1      Header P2  │
     │   (-) BLACK (GND)              │            │  (17-Pin)       (11-Pin)   │
     │                                │            │  ┌──────┐       ┌──────┐   │
     │  [ESC Terminal Header]         │            │  │      │       │      │   │
     │                                │            │  │      │       │      │   │
     │  1. 5V (from RT8279 BEC) ──────┼────────────┼─>│Pin 5 │       │      │   │ (5V Power In / M5V)
     │                                │            │  │(M5V) │       │      │   │
     │  2. GND (Common Return) ───────┼────────────┼─>│Pin 7 │       │      │   │ (GND Ground)
     │                                │            │  │(GND) │       │      │   │
     │  3. E1 (Front-Left Motor) ─────┼────────────┼─>│Pin 13│       │      │   │ (GPIO 5)
     │                                │            │  │(G5)  │       │      │   │
     │  4. E2 (Front-Right Motor) ────┼────────────┼──┼──────┼──────>│Pin 9 │   │ (GPIO 42)
     │                                │            │  │      │       │(G42) │   │
     │  5. E3 (Rear-Left Motor) ──────┼────────────┼─>│Pin 8 │       │      │   │ (GPIO 10)
     │                                │            │  │(G10) │       │      │   │
     │  6. E4 (Rear-Right Motor) ─────┼────────────┼──┼──────┼──────>│Pin 7 │   │ (GPIO 41)
     │                                │            │  │      │       │(G41) │   │
     │  7. VCC (12.6V Battery Raw) ───┼─[CUT/TAPE] │  │      │       │      │   │ (DO NOT CONNECT!)
     │  8. ADC (Current Sense) ───────┼─[OPTIONAL]─┼─>│Pin 17│       │      │   │ (GPIO 1 ADC)
     └────────────────────────────────┘            └──┴──────┴───────┴──────┴───┘
```

### Complete Pin Assignment Table

| Function | Wire Source | Stamp-S3 Connection | ESP32-S3 Pin | Protocol | Notes |
| :--- | :--- | :--- | :---: | :--- | :--- |
| **Main Power** | D92A ESC 5V BEC | Header P1, Pin 5 | **5V (VIN)** | 5.0V DC | Regulated from RT8279 |
| **System Ground** | D92A ESC GND | Header P1, Pin 7 | **GND** | 0V | Common ground return |
| **Motor 1 (FL)** | D92A `E1` | Header P1, Pin 13 | **GPIO 5** | 400Hz PWM | $1000\mu\text{s} - 2000\mu\text{s}$ |
| **Motor 2 (FR)** | D92A `E2` | Header P2, Pin 9 | **GPIO 42** | 400Hz PWM | $1000\mu\text{s} - 2000\mu\text{s}$ |
| **Motor 3 (RL)** | D92A `E3` | Header P1, Pin 8 | **GPIO 10** | 400Hz PWM | $1000\mu\text{s} - 2000\mu\text{s}$ |
| **Motor 4 (RR)** | D92A `E4` | Header P2, Pin 7 | **GPIO 41** | 400Hz PWM | $1000\mu\text{s} - 2000\mu\text{s}$ |
| **Front LiDAR VDD**| Front VL53L3 | Header P2, Pin 1 | **3.3V** | Clean 3.3V | Direct-soldered |
| **Front LiDAR GND**| Front VL53L3 | Header P1, Pin 7 | **GND** | Ground | Direct-soldered |
| **Front LiDAR SDA**| Front VL53L3 | Header P1, Pin 15 | **GPIO 3** | I2C Data | Direct-soldered |
| **Front LiDAR SCL**| Front VL53L3 | Header P1, Pin 14 | **GPIO 4** | I2C Clock | Direct-soldered |
| **Front LiDAR XSHUT**| Front VL53L3| Header P1, Pin 9 | **GPIO 9** | GPIO Out | Active-HIGH enable |
| **RC Receiver RX** | FS-iA10B i-Bus | Port `J2` Pin 1 | **GPIO 1** | UART1 RX | 115,200 baud i-Bus |
| **Down LiDAR RX** | STP23L Serial TX | Port `J3` Pin 1 | **GPIO 15** | UART2 RX | 115,200 baud 1D LiDAR |
| **Onboard ToF** | Internal `U2` Chip| Header P1, Pin 11 | **GPIO 7** | Output LOW | **Permanently disabled** |

---

## ⚠️ Critical Hardware Safety: Diode `D3`

> [!CAUTION]
> On the StampFly PCB, Schottky diode **`D3` (`B5819W SL`)** directly connects `VBAT_IN` to the `+5VOUT` bus.
> **You MUST desolder or snip Diode `D3`**.
> This prevents any high 3S voltage (12.6V) from ever reaching the Stamp-S3 or 5V sensors.

---

## 🎮 Multi-Protocol Radio Support

The firmware includes a modular serial receiver engine configurable via Betaflight Configurator:
* **FlySky i-Bus** (Default): Plug FS-iA10B i-Bus port into **Port `J2`** (`GPIO 1`, `5V`, `GND`). Runs at 115,200 baud with zero latency.
* **S.Bus**: Supported on the same port. Polarity inversion is performed in ESP32-S3 hardware without an external transistor.
* **ExpressLRS (ELRS) / CRSF**: Connect RX to `GPIO 1` and TX to `GPIO 2` for high-refresh EdgeTX telemetry.

---

## 🛠️ Building & Flashing

This project builds using [PlatformIO](https://platformio.org/).

```bash
# Clone the repository
git clone https://github.com/Ojas-sta/wheeltec-f570-stampfly.git
cd wheeltec-f570-stampfly/firmware

# Build firmware
pio run

# Flash to Stamp-S3 over USB-C
pio run --target upload
```

---

## 📖 Detailed Documentation

See the full master blueprint in [`docs/MASTER_PLAN.md`](docs/MASTER_PLAN.md).

---

## 📄 License

MIT License. Developed for open-source autonomous robotics and micro-aerial research.
