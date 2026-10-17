#pragma once

// LilyGO TTGO T-Motor pin map.
// Source: https://github.com/Xinyuan-LilyGO/T-Motor (README "Main Pins Map"
// and Schematic/T-Motor.pdf, commit a867ce9).

#include <stdint.h>

namespace pins {

// TMC2209 control lines
constexpr uint8_t TMC_EN     = 2;   // ENN, active low
constexpr uint8_t TMC_DIR    = 18;
constexpr uint8_t TMC_STEP   = 23;
constexpr uint8_t TMC_CLK    = 19;  // drive low to use the internal clock
constexpr uint8_t TMC_SPREAD = 4;   // high inverts GCONF.en_spreadcycle; we keep it low

// TMC2209 single-wire UART on PDN_UART (TX goes through 1 kOhm)
constexpr uint8_t TMC_UART_RX = 26;
constexpr uint8_t TMC_UART_TX = 27;

// Limit switches on the P1 header (MOSFET build, 2k pull-ups on board).
// Normally-closed to GND: LOW = in travel, HIGH = at limit or wire broken.
// Inputs only: a closed switch would short an output driven high.
// See hw/limit-switches.md.
constexpr uint8_t LIMIT_1 = 33;  // P1 "EN"
constexpr uint8_t LIMIT_2 = 32;  // P1 "DIR"
constexpr uint8_t P1_SPARE = 25; // P1 "STP"

// MT6816 magnetic encoder (SPI), not used yet
constexpr uint8_t ENC_CS   = 15;
constexpr uint8_t ENC_SCLK = 14;
constexpr uint8_t ENC_MISO = 12;
constexpr uint8_t ENC_MOSI = 13;

// OLED / I2C header, not used yet
constexpr uint8_t I2C_SCL = 21;
constexpr uint8_t I2C_SDA = 22;

// Buttons (input-only GPIOs, external pull-ups)
constexpr uint8_t BTN_NEXT  = 36;
constexpr uint8_t BTN_ENTER = 34;
constexpr uint8_t BTN_MENU  = 35;

}  // namespace pins
