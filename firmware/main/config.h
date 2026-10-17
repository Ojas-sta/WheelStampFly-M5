#pragma once

#include <stdint.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

// ==============================================================================
// 1. MOTOR PWM CONFIGURATION (WHEELTEC D92A 4-in-1 ESC)
// ==============================================================================
#define MOTOR_FL_PIN            GPIO_NUM_5   // E1 Front-Left  (CW)
#define MOTOR_FR_PIN            GPIO_NUM_42  // E2 Front-Right (CCW)
#define MOTOR_RL_PIN            GPIO_NUM_10  // E3 Rear-Left   (CCW)
#define MOTOR_RR_PIN            GPIO_NUM_41  // E4 Rear-Right  (CW)

// 400Hz Servo PWM Timings (Period = 2500us, 12-bit resolution: 0 - 4095)
#define MOTOR_PWM_FREQ_HZ       400
#define MOTOR_PWM_RESOLUTION    LEDC_TIMER_12_BIT
#define MOTOR_PWM_MIN_TICKS     1638  // 1000us = Disarmed / Idle (1000/2500 * 4096)
#define MOTOR_PWM_MAX_TICKS     3276  // 2000us = Full Throttle   (2000/2500 * 4096)
#define MOTOR_PWM_RANGE_TICKS   (MOTOR_PWM_MAX_TICKS - MOTOR_PWM_MIN_TICKS)

// ==============================================================================
// 2. RC RECEIVER CONFIGURATION (FlySky FS-iA10B i-Bus on Port J2)
// ==============================================================================
#define RC_UART_NUM             UART_NUM_1
#define RC_RX_PIN               GPIO_NUM_1   // Port J2 Pin 1 (GROVE_I)
#define RC_TX_PIN               GPIO_NUM_2   // Port J2 Pin 2 (GROVE_O)
#define RC_IBUS_BAUDRATE        115200
#define RC_SBUS_BAUDRATE        100000
#define RC_CRSF_BAUDRATE        420000
#define RC_FAILSAFE_TIMEOUT_MS  300          // Failsafe triggers if no frame in 300ms

// ==============================================================================
// 3. DOWNWARD LIDAR CONFIGURATION (WHEELTEC STP23L on Port J3)
// ==============================================================================
#define LIDAR_UART_NUM          UART_NUM_2
#define LIDAR_RX_PIN            GPIO_NUM_15  // Port J3 Pin 1 (SCL line)
#define LIDAR_TX_PIN            GPIO_NUM_13  // Port J3 Pin 2 (SDA line)
#define LIDAR_BAUDRATE          115200

// ==============================================================================
// 4. I2C BUS & FRONT ToF BUMPER (ST VL53L3CX)
// ==============================================================================
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_SDA_IO       GPIO_NUM_3
#define I2C_MASTER_SCL_IO       GPIO_NUM_4
#define I2C_MASTER_FREQ_HZ      400000       // 400kHz Fast Mode

#define FRONT_TOF_XSHUT_PIN     GPIO_NUM_9   // Front ToF Enable (Active HIGH)
#define BOTTOM_TOF_XSHUT_PIN    GPIO_NUM_7   // Onboard ToF Enable (HELD LOW TO DISABLE)

#define BUMPER_WARN_DIST_MM     1000         // Warning threshold: 1.0 meter
#define BUMPER_BRAKE_DIST_MM    500          // Hard reverse-brake threshold: 0.5 meter

// ==============================================================================
// 5. SPI BUS & IMU (Bosch BMI270 & PMW3901 on M5StampFly PCB)
// ==============================================================================
#define IMU_SPI_HOST            SPI2_HOST
#define IMU_SPI_SCK_PIN         GPIO_NUM_44  // StampFly Official SCK
#define IMU_SPI_MOSI_PIN        GPIO_NUM_14  // StampFly Official MOSI
#define IMU_SPI_MISO_PIN        GPIO_NUM_43  // StampFly Official MISO
#define BMI270_CS_PIN           GPIO_NUM_46  // StampFly Official BMI270 CS (PIN_CS in common.h)
#define PMW3901_CS_PIN          GPIO_NUM_12  // StampFly Official PMW3901 CS

// ==============================================================================
// 6. AUDIO / VISUAL HARDWARE
// ==============================================================================
#define BUZZER_GPIO             GPIO_NUM_40  // StampFly Official Buzzer
#define WS2812_RGB_GPIO         GPIO_NUM_39  // StampFly Official WS2812 RGB LED

// ==============================================================================
// 7. TASK PRIORITIES & CORE AFFINITIES
// ==============================================================================
#define FLIGHT_TASK_CORE        1            // Hard Real-Time Core (Zero Jitter)
#define FLIGHT_TASK_PRIORITY    configMAX_PRIORITIES - 1
#define FLIGHT_LOOP_RATE_HZ     1000         // 1kHz Loop (1ms dt)

#define SERVICE_TASK_CORE       0            // Background / Telemetry Core
#define SERVICE_TASK_PRIORITY   5
#define SERVICE_LOOP_RATE_HZ    100          // 100Hz Loop (10ms dt)

#ifdef __cplusplus
}
#endif
