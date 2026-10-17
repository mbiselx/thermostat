#pragma once

#include <cstdint>

#include "esphome/components/uart/uart.h"

namespace esphome::valve_actuator {

// Minimal TMC2209 access over its single-wire UART (PDN_UART), with TX and RX
// tied together on the board, so every byte sent is also received back.
class TMC2209 {
 public:
  static constexpr uint8_t VERSION = 0x21;

  static constexpr uint8_t REG_GCONF = 0x00;
  static constexpr uint8_t REG_GSTAT = 0x01;
  static constexpr uint8_t REG_IOIN = 0x06;
  static constexpr uint8_t REG_IHOLD_IRUN = 0x10;
  static constexpr uint8_t REG_TPOWERDOWN = 0x11;
  static constexpr uint8_t REG_CHOPCONF = 0x6C;
  static constexpr uint8_t REG_DRV_STATUS = 0x6F;
  static constexpr uint8_t REG_PWMCONF = 0x70;

  static constexpr uint8_t GSTAT_RESET = 1 << 0;
  static constexpr uint8_t GSTAT_DRV_ERR = 1 << 1;

  struct Settings {
    float sense_resistor;  // ohm
    float run_current;     // A RMS
    float hold_fraction;   // of run current
    uint16_t microsteps;
    bool spread_cycle;
    bool invert_direction;
  };

  void set_uart(uart::UARTDevice *uart) { this->uart_ = uart; }
  void set_address(uint8_t address) { this->address_ = address & 0b11; }
  uint8_t get_address() const { return this->address_; }

  bool read(uint8_t reg, uint32_t *value);
  void write(uint8_t reg, uint32_t value);

  // True if a TMC2209 answers at the current address.
  bool responds();

  // Writes every register this actuator relies on. Returns false if the
  // driver did not confirm GCONF afterwards.
  bool configure(const Settings &settings);

  // Changes only the run/hold current. Returns the RMS current actually set.
  float set_current(float sense_resistor, float run_current, float hold_fraction);

  // RMS current from the last configure()/set_current(), in A.
  float get_actual_current() const { return this->actual_current_; }

 protected:
  void drain_();

  uart::UARTDevice *uart_{nullptr};
  uint8_t address_{0};
  float actual_current_{0.0f};
  uint32_t chopconf_{0};
};

}  // namespace esphome::valve_actuator
