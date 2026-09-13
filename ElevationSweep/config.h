// Pins and mechanical constants for the elevation axis bench test.
// Board: Adafruit ESP32-S3 Feather, 4MB flash / 2MB PSRAM (Adafruit #5477)
// Arduino board: "Adafruit Feather ESP32-S3 2MB PSRAM"
//
// The silkscreen label is what is printed on the Feather; the GPIO number
// is what the chip calls it. Both are given because the pins_arduino.h
// names (A0, SCK, ...) are what the code uses, while you wire by label.
#pragma once

#include <Arduino.h>

// ---- I2C (STEMMA QT connector): IMU + display. Already wired. ----
constexpr int PIN_SDA = SDA;              // "SDA" = GPIO3
constexpr int PIN_SCL = SCL;              // "SCL" = GPIO4
constexpr int PIN_I2C_PWR = PIN_I2C_POWER;// GPIO7: STEMMA QT rail load switch, must be HIGH

constexpr uint8_t IMU_I2C_ADDR = 0x69;    // ICM-20948 default (0x68 with the jumper closed)
constexpr uint8_t OLED_I2C_ADDR = 0x3D;   // SH1107 1.12" 128x128 default (0x3C with jumper)
constexpr int OLED_W = 128;
constexpr int OLED_H = 128;
constexpr int OLED_ROTATION = 0;          // 0..3, change if the text is sideways

// ---- SPI (hardware bus on the header): AS5048A encoder(s) ----
constexpr int PIN_SPI_SCK  = SCK;         // "SCK"  = GPIO36 -> AS5048A CLK
constexpr int PIN_SPI_MOSI = MOSI;        // "MO"   = GPIO35 -> AS5048A MOSI
constexpr int PIN_SPI_MISO = MISO;        // "MI"   = GPIO37 <- AS5048A MISO
constexpr int PIN_EL_CS    = A0;          // "A0"   = GPIO18 -> AS5048A CSn (elevation)
// Reserved for the azimuth encoder later: "A1" = GPIO17.

// ---- Stepper driver (Adafruit TMC2209 breakout #6121), elevation ----
// Note: Adafruit's own example wires DIR to 5 and STEP to 6; this project
// is the other way round. Wire by this table, not by their picture.
constexpr int PIN_EL_STEP = A8;           // "5"  = GPIO5  -> TMC2209 STEP
constexpr int PIN_EL_DIR  = A9;           // "6"  = GPIO6  -> TMC2209 DIR
constexpr int PIN_EL_EN   = A10;          // "9"  = GPIO9  -> TMC2209 EN (high = outputs off)
constexpr int PIN_EL_DIAG = A2;           // "A2" = GPIO16 <- TMC2209 DIAG (optional; pulled down, so unwired reads "no fault")
// Reserved for the azimuth driver later: STEP/DIR/EN on "10" "11" "12" =
// GPIO10/11/12, DIAG on "A3" = GPIO15. Reserved for TMC2209 UART later:
// "TX" GPIO39 (through 1k) and "RX" GPIO38, shared by both drivers.

// ---- Mechanics ----
constexpr float GEAR_RATIO = 16.0f;       // wireless-actuator gearbox
constexpr int MOTOR_STEPS_PER_REV = 200;  // 1.8 deg NEMA17
constexpr int MICROSTEPS = 16;            // TMC2209 MS1=MS2=HIGH (open = 1/8, which would halve every angle)

// ---- Angle convention ----
// Positions are degrees of elevation at the output shaft:
//   0 = one horizon, 90 = zenith, 180 = the opposite horizon.
// That frame is NOT derivable from the sensors alone - see the README section
// "The one thing the sensors cannot work out for themselves". Establish it
// once per mount with the 'E' serial command, which stores it in flash.

// ---- Motion limits at the output shaft ----
// 0 to 15 deg/s takes ~1.9 s and ~14 deg of travel at each end of a move.
constexpr float MAX_SPEED_DEG_S = 15.0f;
constexpr float ACCEL_DEG_S2 = 8.0f;
constexpr float SOFT_MIN_DEG = -5.0f;     // a little past each horizon
constexpr float SOFT_MAX_DEG = 185.0f;

// ---- Sweep demo ----
constexpr float SWEEP_TO_DEG = 180.0f;    // the far horizon
constexpr uint32_t SWEEP_DWELL_MS = 3000;

// ---- Boot behaviour ----
// true: boot stops in Idle with the driver DISABLED and moves nothing until
// you type 'B'. This is the right setting while the mechanics are unproven -
// a board that resets (or that you reflash) will not swing the arm at you.
// Set false once the axis is trusted and you want the sweep on every boot.
constexpr bool SAFE_BOOT = true;

// ---- Calibration ----
constexpr float DISCOVERY_JOG_DEG = 4.0f; // small open-loop jog to learn signs
constexpr float ASK_UP_JOG_DEG = 10.0f;   // jog shown to the user for the up/down question
constexpr float LEVEL_ABORT_DEG = 60.0f;  // refuse to auto-level if further off than this
constexpr float LEVEL_DONE_DEG = 0.3f;

// If you already know the answers, preset them here to skip the questions.
// -1 = ask/measure, 0 = false, 1 = true.
constexpr int PRESET_UP_IS_POSITIVE = -1;
