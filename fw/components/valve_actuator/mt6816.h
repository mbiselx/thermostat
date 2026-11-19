#pragma once

#include "esphome/core/defines.h"
#ifdef USE_SPI

#include <cstdint>

#include "esphome/components/spi/spi.h"

namespace esphome::valve_actuator {

// MT6816 14-bit magnetic angle sensor on the T-Motor, facing a magnet on the
// back of the motor shaft. SPI mode 3, MSB first. A register read is the
// address with bit 7 set, then one data byte (as in LilyGO's mt6816 example).
//   0x03: angle[13:6]
//   0x04: angle[5:0], bit 1 = magnet too weak, bit 0 = parity
class MT6816 : public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_HIGH, spi::CLOCK_PHASE_TRAILING,
                                     spi::DATA_RATE_1MHZ> {
 public:
  static constexpr int32_t COUNTS_PER_TURN = 16384;

  enum class Result : uint8_t { OK, NO_MAGNET, BAD_PARITY };

  void setup() { this->spi_setup(); }
  Result read(uint16_t *angle);

 protected:
  uint8_t read_register_(uint8_t reg);

  // The parity bit is checked (even parity over both bytes) once the first
  // readings show it holds, and ignored if it doesn't.
  enum class Parity : uint8_t { LEARNING, EVEN, IGNORED };
  Parity parity_{Parity::LEARNING};
  uint8_t parity_reads_{0};
  uint8_t parity_passes_{0};
};

}  // namespace esphome::valve_actuator

#endif  // USE_SPI
