#include "tmc2209.h"

#include <cmath>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::valve_actuator {

static const char *const TAG = "valve_actuator.tmc2209";

static constexpr uint8_t SYNC = 0x05;
static constexpr uint8_t MASTER_ADDRESS = 0xFF;
static constexpr uint8_t WRITE_BIT = 0x80;
static constexpr uint32_t REPLY_TIMEOUT_US = 5000;

// Register values below are the ones the PlatformIO bring-up firmware ran on
// the T-Motor (TMCStepper defaults plus our changes), see the torque-test branch.
static constexpr uint32_t GCONF_EN_SPREADCYCLE = 1 << 2;
static constexpr uint32_t GCONF_SHAFT = 1 << 3;
static constexpr uint32_t GCONF_PDN_DISABLE = 1 << 6;      // PDN_UART is the UART
static constexpr uint32_t GCONF_MSTEP_REG_SELECT = 1 << 7; // microsteps from MRES, not MS1/MS2
static constexpr uint32_t GCONF_MULTISTEP_FILT = 1 << 8;
// I_scale_analog (bit 0) stays 0: current comes from IRUN/IHOLD, not the VREF trimmer.

static constexpr uint32_t CHOPCONF_BASE = 0x10000053;  // toff=3, hstrt=5, hend=0, tbl=0, intpol=1
static constexpr uint32_t CHOPCONF_MRES_MASK = 0x0F000000;
static constexpr uint8_t CHOPCONF_MRES_SHIFT = 24;
static constexpr uint32_t CHOPCONF_VSENSE = 1 << 17;
static constexpr uint32_t PWMCONF_DEFAULT = 0xC10D0024;
static constexpr uint32_t TPOWERDOWN_DEFAULT = 20;  // ~0.3 s from standstill to hold current
static constexpr uint32_t IHOLDDELAY = 1;
// StallGuard runs at every speed; SG_RESULT is polled, DIAG isn't wired to the ESP32.
static constexpr uint32_t TCOOLTHRS_MAX = 0xFFFFF;

// Full-scale sense voltage with vsense=0 and vsense=1 (datasheet, 0.325 V / 0.180 V).
static constexpr float VFS_LOW_SENSE = 0.325f;
static constexpr float VFS_HIGH_SENSE = 0.180f;
static constexpr float SQRT2 = 1.41421356f;

static uint8_t crc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    uint8_t byte = data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if ((crc >> 7) ^ (byte & 0x01)) {
        crc = (crc << 1) ^ 0x07;
      } else {
        crc <<= 1;
      }
      byte >>= 1;
    }
  }
  return crc;
}

void TMC2209::drain_() {
  uint8_t byte;
  while (this->uart_->available() > 0)
    this->uart_->read_byte(&byte);
}

void TMC2209::write(uint8_t reg, uint32_t value) {
  uint8_t frame[8] = {SYNC,
                      this->address_,
                      static_cast<uint8_t>(reg | WRITE_BIT),
                      static_cast<uint8_t>(value >> 24),
                      static_cast<uint8_t>(value >> 16),
                      static_cast<uint8_t>(value >> 8),
                      static_cast<uint8_t>(value),
                      0};
  frame[7] = crc8(frame, 7);
  this->pending_ = false;  // a pending reply would be lost in the drain
  this->drain_();
  this->uart_->write_array(frame, sizeof(frame));
  this->uart_->flush();
  // Let our own echo arrive, then throw it away.
  delayMicroseconds(200);
  this->drain_();
}

void TMC2209::request(uint8_t reg) {
  uint8_t frame[4] = {SYNC, this->address_, reg, 0};
  frame[3] = crc8(frame, 3);
  this->drain_();
  this->uart_->write_array(frame, sizeof(frame));  // 4 bytes fit the TX FIFO, so this doesn't wait
  this->pending_ = true;
  this->pending_reg_ = reg;
  this->pending_len_ = 0;
  this->pending_since_us_ = micros();
}

TMC2209::Reply TMC2209::poll_reply(uint32_t *value) {
  if (!this->pending_)
    return Reply::NONE;
  // The 4-byte echo of the request comes back first, then the 8-byte reply.
  // Scan for the reply instead of counting on the echo.
  while (this->uart_->available() > 0 && this->pending_len_ < sizeof(this->pending_buf_)) {
    this->uart_->read_byte(&this->pending_buf_[this->pending_len_++]);
    if (this->pending_len_ < 8)
      continue;
    const uint8_t *frame = &this->pending_buf_[this->pending_len_ - 8];
    if (frame[0] == SYNC && frame[1] == MASTER_ADDRESS && frame[2] == this->pending_reg_ &&
        crc8(frame, 7) == frame[7]) {
      *value = (uint32_t(frame[3]) << 24) | (uint32_t(frame[4]) << 16) | (uint32_t(frame[5]) << 8) | frame[6];
      this->pending_ = false;
      return Reply::OK;
    }
  }
  if (this->pending_len_ >= sizeof(this->pending_buf_) || micros() - this->pending_since_us_ > REPLY_TIMEOUT_US) {
    this->pending_ = false;
    return Reply::FAILED;
  }
  return Reply::WAITING;
}

bool TMC2209::read(uint8_t reg, uint32_t *value) {
  this->request(reg);
  while (true) {
    switch (this->poll_reply(value)) {
      case Reply::WAITING:
        delayMicroseconds(50);
        break;
      case Reply::OK:
        return true;
      default:
        return false;
    }
  }
}

bool TMC2209::responds() {
  uint32_t ioin;
  return this->read(REG_IOIN, &ioin) && (ioin >> 24) == VERSION;
}

float TMC2209::set_current(float sense_resistor, float run_current, float hold_fraction) {
  // I_rms = (CS + 1) / 32 * V_fs / (R_sense + 0.02 ohm) / sqrt(2)
  auto cs_for = [&](float vfs) { return 32.0f * SQRT2 * run_current * (sense_resistor + 0.02f) / vfs - 1.0f; };
  float cs = cs_for(VFS_LOW_SENSE);
  bool vsense = cs < 16.0f;  // use the high-sensitivity range for better resolution at low current
  if (vsense)
    cs = cs_for(VFS_HIGH_SENSE);
  uint8_t irun = static_cast<uint8_t>(std::fmax(0.0f, std::fmin(31.0f, cs)));
  uint8_t ihold = static_cast<uint8_t>(irun * hold_fraction);

  uint32_t chopconf = vsense ? (this->chopconf_ | CHOPCONF_VSENSE) : (this->chopconf_ & ~CHOPCONF_VSENSE);
  if (chopconf != this->chopconf_) {
    this->chopconf_ = chopconf;
    this->write(REG_CHOPCONF, chopconf);
  }
  this->write(REG_IHOLD_IRUN, ihold | (uint32_t(irun) << 8) | (IHOLDDELAY << 16));

  float vfs = vsense ? VFS_HIGH_SENSE : VFS_LOW_SENSE;
  this->actual_current_ = (irun + 1) / 32.0f * vfs / (sense_resistor + 0.02f) / SQRT2;
  return this->actual_current_;
}

void TMC2209::set_spread_cycle(bool spread_cycle) {
  uint32_t gconf = spread_cycle ? (this->gconf_ | GCONF_EN_SPREADCYCLE) : (this->gconf_ & ~GCONF_EN_SPREADCYCLE);
  if (gconf == this->gconf_)
    return;
  this->gconf_ = gconf;
  this->write(REG_GCONF, gconf);
}

bool TMC2209::configure(const Settings &settings) {
  this->write(REG_GSTAT, GSTAT_RESET | GSTAT_DRV_ERR | 0b100);  // clear latched flags

  uint32_t gconf = GCONF_PDN_DISABLE | GCONF_MSTEP_REG_SELECT | GCONF_MULTISTEP_FILT;
  if (settings.spread_cycle)
    gconf |= GCONF_EN_SPREADCYCLE;
  if (settings.invert_direction)
    gconf |= GCONF_SHAFT;
  this->gconf_ = gconf;
  this->write(REG_GCONF, gconf);

  // MRES: 0 = 256 microsteps ... 8 = full steps.
  uint32_t mres = 8;
  for (uint16_t steps = settings.microsteps; steps > 1; steps >>= 1)
    mres--;
  this->chopconf_ = (CHOPCONF_BASE & ~CHOPCONF_MRES_MASK) | (mres << CHOPCONF_MRES_SHIFT);
  this->write(REG_CHOPCONF, this->chopconf_);
  this->write(REG_PWMCONF, PWMCONF_DEFAULT);
  this->write(REG_TPOWERDOWN, TPOWERDOWN_DEFAULT);
  this->write(REG_TCOOLTHRS, TCOOLTHRS_MAX);
  this->set_current(settings.sense_resistor, settings.run_current, settings.hold_fraction);

  uint32_t readback;
  if (!this->read(REG_GCONF, &readback) || (readback & 0x3FF) != gconf) {
    ESP_LOGE(TAG, "GCONF readback failed (wrote 0x%03X)", (unsigned) gconf);
    return false;
  }
  return true;
}

}  // namespace esphome::valve_actuator
