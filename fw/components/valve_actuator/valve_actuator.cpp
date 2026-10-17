#include "valve_actuator.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "esphome/core/log.h"

namespace esphome::valve_actuator {

static const char *const TAG = "valve_actuator";

static constexpr uint32_t DRIVER_POLL_MS = 1000;
static constexpr uint32_t PUBLISH_INTERVAL_MS = 500;

// DRV_STATUS flags worth a warning (open-load flags misfire at standstill).
static constexpr uint32_t DRV_OTPW = 1 << 0;
static constexpr uint32_t DRV_OT = 1 << 1;
static constexpr uint32_t DRV_SHORT_MASK = 0b1111 << 2;

void ValveActuator::setup() {
  // Keep the motor unpowered until the driver is configured.
  this->enable_pin_->setup();
  this->enable_pin_->digital_write(false);
  this->step_pin_->setup();
  this->step_pin_->digital_write(false);
  this->dir_pin_->setup();
  this->dir_pin_->digital_write(false);
  if (this->clk_pin_ != nullptr) {
    this->clk_pin_->setup();
    this->clk_pin_->digital_write(false);  // low = internal clock
  }
  if (this->spread_pin_ != nullptr) {
    this->spread_pin_->setup();
    this->spread_pin_->digital_write(false);  // high would invert GCONF's chopper mode
  }

  this->steps_per_deg_ = this->steps_per_revolution_ * this->settings_.microsteps * this->gear_ratio_ / 360.0f;
  this->travel_steps_ = lroundf(this->travel_deg_ * this->steps_per_deg_);
  this->max_speed_ = this->max_speed_deg_s_ * this->steps_per_deg_;
  this->accel_ = this->accel_deg_s2_ * this->steps_per_deg_;
  this->min_speed_ = sqrtf(2.0f * this->accel_);  // speed after one step from rest

  // There is no homing yet, so trust the position saved after the last move.
  auto restored = this->restore_state_();
  this->position = restored.has_value() ? restored->position : valve::VALVE_CLOSED;
  this->position_ = this->target_ = lroundf(this->position * this->travel_steps_);
  this->current_operation = valve::VALVE_OPERATION_IDLE;
  this->publish_state(false);

  this->driver_.set_uart(this);
  if (!this->connect_driver_()) {
    ESP_LOGW(TAG, "No TMC2209 answering yet, check the 9-12 V motor supply");
    this->status_set_warning();
  }
}

void ValveActuator::loop() {
  if (this->moving_) {
    this->run_motion_();
    return;
  }
  // UART register access blocks for a few ms, so only poll between moves.
  uint32_t now = millis();
  if (now - this->last_driver_poll_ >= DRIVER_POLL_MS) {
    this->last_driver_poll_ = now;
    this->poll_driver_();
  }
}

void ValveActuator::dump_config() {
  LOG_VALVE("", "Valve actuator", this);
  LOG_PIN("  Step Pin: ", this->step_pin_);
  LOG_PIN("  Dir Pin: ", this->dir_pin_);
  LOG_PIN("  Enable Pin: ", this->enable_pin_);
  LOG_PIN("  CLK Pin: ", this->clk_pin_);
  LOG_PIN("  SPREAD Pin: ", this->spread_pin_);
  ESP_LOGCONFIG(TAG,
                "  Driver: %s (UART address %u)\n"
                "  Run current: %.0f mA requested, %.0f mA set\n"
                "  Hold current: %.0f%%\n"
                "  Microsteps: %u, gear ratio %.2f, %.2f steps/deg\n"
                "  Travel: %.1f deg (%d steps)\n"
                "  Max speed: %.1f deg/s, acceleration %.1f deg/s^2\n"
                "  Chopper: %s, %s when idle",
                this->driver_ok_ ? "connected" : "not found", this->driver_.get_address(),
                this->settings_.run_current * 1000, this->driver_.get_actual_current() * 1000,
                this->settings_.hold_fraction * 100, this->settings_.microsteps, this->gear_ratio_,
                this->steps_per_deg_, this->travel_deg_, (int) this->travel_steps_, this->max_speed_deg_s_,
                this->accel_deg_s2_, this->settings_.spread_cycle ? "SpreadCycle" : "StealthChop",
                this->hold_when_idle_ ? "holding" : "unpowered");
  LOG_BINARY_SENSOR("  ", "Open limit", this->open_limit_);
  LOG_BINARY_SENSOR("  ", "Closed limit", this->closed_limit_);
}

valve::ValveTraits ValveActuator::get_traits() {
  valve::ValveTraits traits;
  traits.set_supports_position(true);
  traits.set_supports_stop(true);
  return traits;
}

void ValveActuator::control(const valve::ValveCall &call) {
  if (call.get_stop()) {
    this->stop_();
    return;
  }
  auto position = call.get_position();
  if (position.has_value())
    this->move_to_(lroundf(std::clamp(*position, 0.0f, 1.0f) * this->travel_steps_));
}

// --- TMC2209 -----------------------------------------------------------------

bool ValveActuator::connect_driver_() {
  for (uint8_t address = 0; address < 4; address++) {
    if (this->fixed_address_.has_value() && address != *this->fixed_address_)
      continue;
    this->driver_.set_address(address);
    if (!this->driver_.responds())
      continue;
    if (!this->driver_.configure(this->settings_))
      return false;
    this->driver_ok_ = true;
    this->status_clear_warning();
    ESP_LOGI(TAG, "TMC2209 found at UART address %u, %.0f mA RMS", address,
             this->driver_.get_actual_current() * 1000);
    if (this->hold_when_idle_)
      this->enable_pin_->digital_write(true);
    return true;
  }
  return false;
}

// Notice the driver vanishing (motor supply off) or resetting (a brown-out
// wipes its registers), and pick it back up when it returns.
void ValveActuator::poll_driver_() {
  if (!this->driver_ok_) {
    this->connect_driver_();
    return;
  }
  uint32_t gstat;
  if (!this->driver_.read(TMC2209::REG_GSTAT, &gstat)) {
    ESP_LOGW(TAG, "TMC2209 stopped responding (motor supply off?)");
    this->driver_ok_ = false;
    this->enable_pin_->digital_write(false);
    this->status_set_warning();
    return;
  }
  if (gstat & TMC2209::GSTAT_RESET) {
    ESP_LOGW(TAG, "TMC2209 reset, reapplying configuration");
    this->driver_.configure(this->settings_);
  } else if (gstat & TMC2209::GSTAT_DRV_ERR) {
    ESP_LOGW(TAG, "TMC2209 reported a driver error (overtemperature or short)");
    this->log_driver_status_();
    this->driver_.write(TMC2209::REG_GSTAT, TMC2209::GSTAT_DRV_ERR);
  }
}

void ValveActuator::log_driver_status_() {
  uint32_t status;
  if (!this->driver_.read(TMC2209::REG_DRV_STATUS, &status))
    return;
  if (status & (DRV_OTPW | DRV_OT | DRV_SHORT_MASK)) {
    ESP_LOGW(TAG, "DRV_STATUS 0x%08X:%s%s%s", (unsigned) status, (status & DRV_OTPW) ? " overtemp-warning" : "",
             (status & DRV_OT) ? " overtemp-shutdown" : "", (status & DRV_SHORT_MASK) ? " short" : "");
  } else {
    ESP_LOGD(TAG, "DRV_STATUS 0x%08X", (unsigned) status);
  }
}

// --- Motion ------------------------------------------------------------------

bool ValveActuator::limit_tripped_(int8_t direction) const {
  // Switches are normally closed, so "on" means at the limit or a broken wire.
  binary_sensor::BinarySensor *limit = direction > 0 ? this->open_limit_ : this->closed_limit_;
  return limit != nullptr && limit->state;
}

void ValveActuator::move_to_(int32_t target) {
  if (!this->driver_ok_) {
    ESP_LOGW(TAG, "Ignoring move: no TMC2209 (motor supply off?)");
    this->publish_position_(false);
    return;
  }
  if (target == this->position_ && !this->moving_)
    return;
  int8_t direction = target > this->position_ ? 1 : -1;
  if (this->limit_tripped_(direction)) {
    ESP_LOGW(TAG, "Not moving: %s limit is tripped (at the limit, or a broken wire)",
             direction > 0 ? "open" : "closed");
    this->publish_position_(false);
    return;
  }

  if (this->moving_ && direction != this->direction_)
    this->speed_ = 0;  // reverse from standstill
  this->target_ = target;
  if (!this->moving_) {
    this->enable_pin_->digital_write(true);
    this->moving_ = true;
    this->speed_ = 0;
    this->last_update_us_ = this->last_step_us_ = micros();
    this->high_freq_.start();
  }
  this->current_operation = direction > 0 ? valve::VALVE_OPERATION_OPENING : valve::VALVE_OPERATION_CLOSING;
  this->publish_position_(false);
}

void ValveActuator::stop_() {
  if (!this->moving_)
    return;
  // Decelerate rather than stopping dead.
  int32_t stopping_steps = static_cast<int32_t>(this->speed_ * this->speed_ / (2 * this->accel_));
  int8_t direction = this->target_ > this->position_ ? 1 : -1;
  this->target_ = this->position_ + direction * std::min(stopping_steps, std::abs(this->target_ - this->position_));
}

void ValveActuator::run_motion_() {
  if (this->position_ == this->target_) {
    this->finish_move_();
    return;
  }
  int8_t direction = this->target_ > this->position_ ? 1 : -1;
  if (this->limit_tripped_(direction)) {
    ESP_LOGW(TAG, "%s limit tripped, stopping", direction > 0 ? "Open" : "Closed");
    this->target_ = this->position_;
    this->finish_move_();
    return;
  }

  // Trapezoidal profile: accelerate until the remaining distance is the
  // stopping distance, then decelerate.
  uint32_t now = micros();
  float dt = (now - this->last_update_us_) * 1e-6f;
  this->last_update_us_ = now;
  float remaining = std::abs(this->target_ - this->position_);
  if (remaining <= this->speed_ * this->speed_ / (2 * this->accel_)) {
    this->speed_ -= this->accel_ * dt;
  } else {
    this->speed_ += this->accel_ * dt;
  }
  this->speed_ = std::clamp(this->speed_, this->min_speed_, this->max_speed_);

  if (now - this->last_step_us_ < static_cast<uint32_t>(1e6f / this->speed_))
    return;
  this->last_step_us_ = now;

  if (direction != this->direction_) {
    this->dir_pin_->digital_write(direction > 0);  // high = opening
    this->direction_ = direction;
    delayMicroseconds(5);
  }
  this->step_pin_->digital_write(true);
  delayMicroseconds(2);
  this->step_pin_->digital_write(false);
  this->position_ += direction;

  if (millis() - this->last_publish_ms_ >= PUBLISH_INTERVAL_MS)
    this->publish_position_(false);
}

void ValveActuator::finish_move_() {
  this->moving_ = false;
  this->speed_ = 0;
  this->high_freq_.stop();
  if (!this->hold_when_idle_)
    this->enable_pin_->digital_write(false);
  this->current_operation = valve::VALVE_OPERATION_IDLE;
  this->publish_position_(true);
  ESP_LOGD(TAG, "Stopped at %.1f deg", this->position_ / this->steps_per_deg_);
  this->log_driver_status_();
}

void ValveActuator::publish_position_(bool save) {
  this->position = std::clamp(float(this->position_) / this->travel_steps_, 0.0f, 1.0f);
  this->publish_state(save);
  this->last_publish_ms_ = millis();
}

}  // namespace esphome::valve_actuator
