#include "mt6816.h"

#ifdef USE_SPI

#include "esphome/core/log.h"

namespace esphome::valve_actuator {

static const char *const TAG = "valve_actuator.mt6816";

static constexpr uint8_t REG_ANGLE_HIGH = 0x03;
static constexpr uint8_t REG_ANGLE_LOW = 0x04;
static constexpr uint8_t READ_BIT = 0x80;
static constexpr uint8_t NO_MAGNET_BIT = 0x02;
static constexpr uint8_t PARITY_LEARN_READS = 32;

uint8_t MT6816::read_register_(uint8_t reg) {
  this->enable();
  this->transfer_byte(READ_BIT | reg);
  uint8_t value = this->transfer_byte(0x00);
  this->disable();
  return value;
}

MT6816::Result MT6816::read(uint16_t *angle) {
  uint8_t high = this->read_register_(REG_ANGLE_HIGH);
  uint8_t low = this->read_register_(REG_ANGLE_LOW);
  uint16_t word = (uint16_t(high) << 8) | low;
  bool parity_ok = __builtin_parity(word) == 0;

  switch (this->parity_) {
    case Parity::LEARNING:
      this->parity_passes_ += parity_ok;
      if (++this->parity_reads_ >= PARITY_LEARN_READS) {
        if (this->parity_passes_ >= PARITY_LEARN_READS - 2) {
          this->parity_ = Parity::EVEN;
        } else {
          this->parity_ = Parity::IGNORED;
          ESP_LOGW(TAG, "Parity bit doesn't hold (%u of %u reads), ignoring it", this->parity_passes_,
                   this->parity_reads_);
        }
      }
      break;
    case Parity::EVEN:
      if (!parity_ok)
        return Result::BAD_PARITY;
      break;
    case Parity::IGNORED:
      break;
  }

  if (low & NO_MAGNET_BIT)
    return Result::NO_MAGNET;
  *angle = word >> 2;
  return Result::OK;
}

}  // namespace esphome::valve_actuator

#endif  // USE_SPI
