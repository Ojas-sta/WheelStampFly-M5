# WHEELTEC F570 × M5StampFly Hybrid Drone: Master Architecture & Implementation Plan

**Target Aircraft**: WHEELTEC F570 Quadcopter Frame, 1104 Brushless Motors, D92A 4-in-1 ESC, 3S LiPo  
**Flight Computer & Sensor Deck**: M5Stack Stamp-S3 (ESP32-S3FN8) + M5StampFly Sensor PCB  
**Flight Control Ecosystem**: Custom Betaflight-Compatible Firmware with Full Configurator GUI Support  
**Primary RC System**: FlySky FS-i6S Transmitter + FS-iA10B Receiver over i-Bus (with ELRS / S.Bus switchability)

---

## 1. System Overview & Hardware Topology

The hybrid aircraft marries high-power brushless propulsion with a rich multi-sensor research stack:

```
                          [ 3S LiPo Battery (11.1V - 12.6V) ]
                                          │
                                          ▼
                       +─────────────────────────────────────+
                       │      WHEELTEC D92A 4-in-1 ESC       │
                       │   - 4x Brushless Channels (1104)    │
                       │   - Onboard RT8279 5V BEC           │
                       │   - Heavy Filtering / TVS Diode     │
                       +───────┬─────────────────────┬───────+
               Motor Signals   │                     │  Clean 5.0V & GND
            (E1, E2, E3, E4)   │                     │  (Main System Power)
                               ▼                     ▼
        +───────────────────────────────────────────────────────────────+
        │             M5Stamp-S3 + M5StampFly Sensor Deck              │
        │                                                               │
        │  [M5Stamp-S3] Dual-Core Xtensa LX7 @ 240MHz, 8MB Flash        │
        │  [IMU]        Bosch BMI270 (SPI: SCK 39, MOSI 14, MISO 38)    │
        │  [Barometer]  Bosch BMP280 (I2C: SDA 3, SCL 4)                │
        │  [Compass]    Bosch BMM150 (I2C: SDA 3, SCL 4)                │
        │  [Opt Flow]   PixArt PMW3901 (SPI: CS=12)                     │
        │  [Bottom ToF] VL53L3 (U2) DISABLED: GPIO 7 held LOW          │
        +───────────────┬───────────────────────────────┬───────────────+
                        │                               │
        Direct Solder   │                               │ Port J2 (HY-2.0)
     (GPIO 3, 4, 9, 3.3V)│                               │ (GPIO 1 RX, 5V, GND)
                        ▼                               ▼
       +────────────────────────────────+    +──────────────────────────+
       |   Front VL53L3 Laser LiDAR     |    |  FlySky FS-iA10B (i-Bus) |
       |   Forward Collision Bumper     |    |  10-Channel RC Receiver  |
       +────────────────────────────────+    +──────────────────────────+
                                                        │
                                                        │ Port J3 (HY-2.0)
                                                        │ (GPIO 15 RX, 5V, GND)
                                                        ▼
                                             +──────────────────────────+
                                             |  WHEELTEC STP23L LiDAR   |
                                             |  Downward Altitude Hold  |
                                             +──────────────────────────+
```

---

## 2. Mechanical Integration & Isolation

1. **StampFly PCB Decoupling**:
   * Unscrew the StampFly PCB and Stamp-S3 from the plastic whoop duct frame.
   * **Vibration Isolation**: 1104 brushless motors produce sharp high-frequency vibrations. Mount the StampFly board to the F570 carbon top plate using **M2 silicone rubber gummies / standoffs** or **3M VHB silicone flight controller gel tape**. Never screw it directly onto the bare carbon fiber plate.
2. **Optical Flow Lens Clearance**:
   * The PMW3901 optical flow sensor is soldered to the underside of the StampFly PCB.
   * Ensure the F570 carbon frame has a clear opening beneath the lens, or use standoffs tall enough to give the lens an unobstructed view of the ground.
3. **Front LiDAR Bumper Placement**:
   * Mount the external VL53L3 sensor module on the front nose of the carbon frame facing directly forward.
   * Secure with an M2 standoff or 3D-printed TPU nose mount.

---

## 3. Electrical & Power Architecture

### ⚠️ Critical Power Safety Action: Clip Diode `D3`
On the StampFly schematic, Schottky diode **`D3` (`B5819W SL`)** connects `VBAT_IN` to `+5VOUT`.
* **Action**: **Desolder or clip Diode `D3`.**
* **Rationale**: This guarantees that high 3S voltage (12.6V) can **never** accidentally backfeed into the Stamp-S3 or sensor rails if battery leads are ever plugged into the StampFly battery jack.

```
       D92A ESC Board (5V BEC) ─────────────> +5VOUT (Stamp-S3 Pin 17)
       D92A ESC Board (GND)    ─────────────> GND (Common System Ground)
```

### Complete Pinout & Connection Matrix

| Subsystem | Wire / Net | Physical Source | Connection on Stamp-S3 / PCB | Protocol / Signal |
| :--- | :--- | :--- | :--- | :--- |
| **Power** | 5V BEC | D92A ESC 5V Pad | `+5VOUT` (Stamp-S3 Header P1 Pin 5) | Clean 5.0V $\pm 0.1\text{V}$ |
| **Power** | Ground | D92A ESC GND Pad | `GND` (Common Ground) | 0V Reference |
| **Motor 1** | Front-Left (E1)| D92A ESC `E1` | **GPIO 5** (`L-Up` / Gate of Q4) | 400Hz Servo PWM |
| **Motor 2** | Front-Right (E2)| D92A ESC `E2` | **GPIO 42** (`R-Up` / Gate of Q3) | 400Hz Servo PWM |
| **Motor 3** | Rear-Left (E3) | D92A ESC `E3` | **GPIO 10** (`L-Down` / Gate of Q5) | 400Hz Servo PWM |
| **Motor 4** | Rear-Right (E4)| D92A ESC `E4` | **GPIO 41** (`R-Down` / Gate of Q6)| 400Hz Servo PWM |
| **Front LiDAR**| Power (VDD) | External ToF Board | **`3.3V`** Pin on Stamp-S3 | 3.3V Regulated |
| **Front LiDAR**| Ground (GND)| External ToF Board | **`GND`** Pin on Stamp-S3 | 0V Reference |
| **Front LiDAR**| I2C Data (SDA)| External ToF Board | **GPIO 3** (Stamp-S3 P1-15) | I2C Data (4.7kΩ pullup) |
| **Front LiDAR**| I2C Clock (SCL)| External ToF Board | **GPIO 4** (Stamp-S3 P1-14) | I2C Clock (4.7kΩ pullup) |
| **Front LiDAR**| Enable (XSHUT)| External ToF Board | **GPIO 9** (Stamp-S3 P1-9) | Active-HIGH Power Enable |
| **RC Receiver**| i-Bus Signal | FS-iA10B i-Bus Port | **`J2` Pin 1 (GPIO 1)** | 115,200 baud UART1 RX |
| **RC Receiver**| 5V Power | FS-iA10B VCC | **`J2` Pin 3 (+5VOUT)** | 5.0V Receiver Power |
| **RC Receiver**| Ground | FS-iA10B GND | **`J2` Pin 4 (GND)** | Ground Return |
| **Down LiDAR** | LiDAR Data | STP23L Serial TX | **`J3` Pin 1 (GPIO 15)** | 115,200 baud UART2 RX |
| **Down LiDAR** | 5V Power | STP23L 5V | **`J3` Pin 3 (+5VOUT)** | 5.0V Sensor Power |
| **Down LiDAR** | Ground | STP23L GND | **`J3` Pin 4 (GND)** | Ground Return |
| **Onboard ToF**| Shutdown | Internal `U2` Chip | **GPIO 7 held LOW** | Forces `U2` to sleep |

---

## 4. Custom Betaflight Firmware Architecture

The firmware runs as a lightweight, real-time Betaflight-compatible flight controller on the ESP32-S3:

```
┌────────────────────────────────────────────────────────────────────────┐
│                   Official Betaflight Configurator                     │
│      (PID Tuning, Rates, Modes, Receiver, Motors, CLI, 3D Preview)     │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │ USB-CDC / Serial (MSP V2 Protocol)
┌───────────────────────────────────▼────────────────────────────────────┐
│                    Custom ESP32-S3 Betaflight Core                     │
│                                                                        │
│  ├── [MSP Server]         MultiWii Serial Protocol V2                  │
│  ├── [Config Engine]      NVS Flash Storage (PIDs, Rates, Features)    │
│  ├── [Flight Mixer]       Quad-X Brushless Mixer on LEDC (400Hz)       │
│  ├── [Attitude Estimator] Madgwick / Mahony Fusion @ 500Hz             │
│  └── [Safety Governor]    Failsafe Watchdog & Collision Bumper         │
└───────────────────────┬────────────────────────┬───────────────────────┘
                        │                        │
       ┌────────────────▼────────┐      ┌────────▼────────────────┐
       │   Sensor Drivers (HAL)  │      │  Multi-Protocol RC Link │
       │                         │      │                         │
       │ - BMI270 (SPI 10MHz)    │      │ - FlySky i-Bus (115.2k) │
       │ - BMP280 Baro (I2C)     │      │ - S.Bus (100k Inverted) │
       │ - BMM150 Mag (I2C)      │      │ - ELRS / CRSF (420k)    │
       │ - PMW3901 Flow (SPI)    │      │ - Atom Joystick ESP-NOW │
       │ - Front VL53L3 ToF (I2C)│      └─────────────────────────┘
       │ - STP23L 1D LiDAR (UART)│
       └─────────────────────────┘
```

### Key Firmware Capabilities

1. **400 Hz Brushless ESC Timing**:
   * ESP32 LEDC Timer configured at 400 Hz (period $2500\ \mu\text{s}$) with 12-bit resolution ($0 - 4095$).
   * $1000\ \mu\text{s}$ (Disarmed / Idle) = **1638 ticks**.
   * $2000\ \mu\text{s}$ (100% Throttle) = **3276 ticks**.
2. **Multi-Protocol Receiver Engine**:
   * In Betaflight Configurator **Receiver Tab**, the pilot selects:
     * `IBUS` $\rightarrow$ FS-i6S + FS-iA10B (Default)
     * `SBUS` $\rightarrow$ S.Bus with software inversion enabled in ESP32 UART silicon
     * `CRSF` $\rightarrow$ ExpressLRS / EdgeTX
     * `ESPNOW` $\rightarrow$ Direct link to M5 Atom Joystick
3. **Active Collision Avoidance Bumper (`BUMPER` Mode)**:
   * Accessible as an AUX mode switch in the Betaflight **Modes Tab**.
   * When enabled: If the front LiDAR detects an obstacle closer than $0.6\text{m}$, the controller overrides the forward pitch stick and injects reverse tilt to stop before impact.
4. **Flexible ToF Sensor Role Selection (Betaflight CLI)**:
   ```text
   set tof_altitude_source = STP23L_UART  # Ground distance from WHEELTEC LiDAR
   set tof_front_role      = COLLISION_SHIELD
   set tof_bottom_internal = DISABLED     # Keep GPIO 7 LOW
   save
   ```

---

## 5. Step-by-Step Build & Verification Checklist

### Phase 1: Hardware Preparation
- [ ] **Desolder / Cut Diode `D3`** on the StampFly PCB.
- [ ] Inspect the bottom optical flow lens and verify clearance on the F570 frame.
- [ ] Direct-solder the 5 wires (`3.3V`, `GND`, `SDA/GPIO 3`, `SCL/GPIO 4`, `XSHUT/GPIO 9`) from the Stamp-S3 to the front VL53L3 sensor module with twisted pairs.
- [ ] Mount the StampFly board using M2 silicone vibration standoffs.

### Phase 2: Power & ESC Wiring
- [ ] Solder D92A ESC 5V BEC and GND to StampFly `+5VOUT` and `GND`.
- [ ] Solder the 4 ESC signal wires (`E1–E4`) to the motor pads (GPIO 5, 42, 10, 41).
- [ ] **Bench Safety**: **REMOVE ALL PROPELLERS.**
- [ ] Connect the 3S battery to the D92A ESC. Verify the Stamp-S3 powers on and the ESC produces normal startup beeps.

### Phase 3: Receiver & Sensor Hookup
- [ ] Plug the FS-iA10B i-Bus lead into Port `J2`. Bind to the FS-i6S transmitter.
- [ ] Plug the STP23L downward LiDAR into Port `J3`.
- [ ] Connect the Stamp-S3 via USB-C to your laptop. Open **Betaflight Configurator**.
- [ ] Verify the 3D model tilts accurately in the Setup tab.
- [ ] Verify all 10 channels move smoothly in the Receiver tab under `IBUS`.
- [ ] Test motor rotation direction in the Motors tab (Quad-X format).

---

## 6. Outcome

This architecture delivers a complete, professional-grade autonomous research platform combining the robust 3S brushless power of the WHEELTEC F570 with the multi-sensor intelligence of the M5StampFly, fully configurable via the standard Betaflight desktop GUI.
