#pragma once

// Compile-time defaults. Current, speed, acceleration and chopper mode can
// also be changed at runtime over the serial console (type `help`).

#include <stdint.h>

namespace config {

// --- Driver hardware -------------------------------------------------------

// Sense resistors on BRA/BRB are 120 mOhm (R7/R8 in the T-Motor schematic).
// LilyGO's examples use 0.11, which reads ~8% low on current.
constexpr float R_SENSE = 0.12f;

// With 120 mOhm the TMC2209 tops out at ~1640 mA RMS (CS=31, vsense=0).
// The board has no heatsink, so long moves near the limit will get hot.
constexpr uint16_t MAX_RUN_CURRENT_MA = 1600;

// UART address set by the MS1/MS2 DIP switch (both off = 0b11).
// -1 scans all four addresses at boot and uses the first that answers.
constexpr int8_t DRIVER_ADDRESS = -1;

constexpr uint32_t TMC_UART_BAUD = 115200;

// --- Motor ------------------------------------------------------------------

constexpr uint16_t FULL_STEPS_PER_REV = 200;  // 1.8 deg NEMA17
constexpr uint16_t MICROSTEPS = 16;           // interpolated to 256 internally

// Motor revolutions per valve revolution (1.0 for direct drive).
constexpr float GEAR_RATIO = 1.0f;

// Flip if "open" turns the wrong way.
constexpr bool INVERT_DIRECTION = false;

// --- Defaults for a stiff rotary valve -------------------------------------

// Stepper torque falls off with speed, so a stiff valve wants high current,
// low speed and gentle acceleration.
constexpr uint16_t RUN_CURRENT_MA = 1000;
constexpr float HOLD_CURRENT_FRACTION = 0.3f;  // of run current, while enabled

constexpr float MAX_SPEED_DEG_S = 90.0f;   // valve degrees per second
constexpr float ACCEL_DEG_S2 = 180.0f;     // valve degrees per second^2

// SpreadCycle keeps torque up under load; StealthChop is quieter but can
// lose steps on a stiff valve without tuning.
constexpr bool USE_SPREADCYCLE = true;

// Cut motor power after each move. The valve's own friction holds it, and
// this keeps the board and motor cool when idle.
constexpr bool DISABLE_WHEN_IDLE = true;

}  // namespace config
