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

// StealthChop tunes its current regulation at standstill and full current
// before the first move (datasheet "AT#1", at least 130 ms).
static constexpr uint32_t SETTLE_MS = 250;
// Valve degrees moved open first when calibration starts near the closed end,
// so the seek has room to measure free running before it reaches the stop.
static constexpr float RUNUP_DEG = 15.0f;
// SG_RESULT is updated once per full step: poll about that often, but no faster than this.
static constexpr uint32_t SG_MIN_INTERVAL_US = 10000;
static constexpr uint32_t SG_REPORT_MS = 250;
static constexpr uint8_t SG_SKIP_SAMPLES = 4;      // readings ignored once at speed
static constexpr uint8_t SG_BASELINE_SAMPLES = 8;  // averaged into the free-running value
static constexpr uint8_t SG_STALL_SAMPLES = 3;     // consecutive low readings that make a stall
static constexpr uint8_t SG_MAX_FAILURES = 10;     // missed replies before the driver counts as gone
static constexpr float SG_USABLE_BASELINE = 10.0f;
// Measured travel is saved in motor degrees, so it survives gear_ratio and microstep changes.
static constexpr uint32_t TRAVEL_PREF_VERSION = 0x54524156;  // "TRAV"

static const char *const STATUS_NO_DRIVER = "No motor supply";
static const char *const STATUS_CALIBRATING = "Calibrating";
static const char *const STATUS_READY = "Ready";
static const char *const STATUS_NOT_CALIBRATED = "Not calibrated";
static const char *const STATUS_NO_CLOSED_END = "Closed end not found";
static const char *const STATUS_NO_OPEN_END = "Open end not found";

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
  this->max_speed_ = this->max_speed_deg_s_ * this->steps_per_deg_;
  this->accel_ = this->accel_deg_s2_ * this->steps_per_deg_;
  this->min_speed_ = sqrtf(2.0f * this->accel_);  // speed after one step from rest

  // Travel: as configured, else as last measured, else the search limit until measured.
  this->calibration_result_ = STATUS_NOT_CALIBRATED;
  this->travel_pref_ = this->make_entity_preference<float>(TRAVEL_PREF_VERSION);
  bool travel_known = this->travel_configured_;
  if (!travel_known) {
    float motor_deg;
    if (this->travel_pref_.load(&motor_deg) && motor_deg > 0) {
      this->travel_deg_ = motor_deg / this->gear_ratio_;
      travel_known = true;
    } else {
      this->travel_deg_ = this->max_travel_deg_;
      ESP_LOGW(TAG, "Valve travel not measured yet; calibration will measure it");
    }
  }
  this->travel_steps_ = std::max<int32_t>(1, static_cast<int32_t>(lroundf(this->travel_deg_ * this->steps_per_deg_)));
  if (travel_known && this->travel_sensor_ != nullptr)
    this->travel_sensor_->publish_state(this->travel_deg_);

  // Until calibrated, trust the position saved after the last move.
  auto restored = this->restore_state_();
  this->position = restored.has_value() ? restored->position : valve::VALVE_CLOSED;
  this->position_ = this->target_ = static_cast<int32_t>(lroundf(this->position * this->travel_steps_));
  this->current_operation = valve::VALVE_OPERATION_IDLE;
  this->publish_state(false);

  if (this->homing_enabled_ && this->home_on_boot_) {
    // Runs from loop(), once every component is set up (template numbers may
    // change the threshold or speed during their setup).
    this->calibration_requested_ = true;
    this->resume_position_ = this->position;
  }

  this->driver_.set_uart(this);
  if (!this->connect_driver_()) {
    ESP_LOGW(TAG, "No TMC2209 answering yet, check the 9-12 V motor supply");
    this->status_set_warning();
    this->set_status_(STATUS_NO_DRIVER);
  }
}

void ValveActuator::loop() {
  if (this->moving_) {
    this->run_motion_();
    if (this->moving_ && this->sg_active_)
      this->poll_stallguard_();
    return;
  }
  if (this->homing_ == Homing::SETTLE) {
    if (millis() - this->settle_start_ms_ >= SETTLE_MS)
      this->calibration_step_();
    return;
  }
  if (this->calibration_requested_ && this->driver_ok_) {
    this->start_calibration_();
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
                "  Microsteps: %u, gear ratio %.3f, %.2f steps/deg\n"
                "  Travel: %.1f deg (%d steps), %s\n"
                "  Max speed: %.1f deg/s, acceleration %.1f deg/s^2\n"
                "  Chopper: %s, %s when idle",
                this->driver_ok_ ? "connected" : "not found", this->driver_.get_address(),
                this->settings_.run_current * 1000, this->driver_.get_actual_current() * 1000,
                this->settings_.hold_fraction * 100, this->settings_.microsteps, this->gear_ratio_,
                this->steps_per_deg_, this->travel_deg_, (int) this->travel_steps_,
                this->travel_configured_ ? "configured" : "measured by calibration", this->max_speed_deg_s_,
                this->accel_deg_s2_, this->settings_.spread_cycle ? "SpreadCycle" : "StealthChop",
                this->hold_when_idle_ ? "holding" : "unpowered");
  if (this->homing_enabled_) {
    ESP_LOGCONFIG(TAG,
                  "  Calibration: %s, %.1f deg/s, %.0f mA\n"
                  "  Stall: StallGuard below %.0f%% of free running, searching up to %.0f deg, back-off %.1f deg",
                  this->home_on_boot_ ? "on boot" : "on request", this->homing_speed_deg_s_,
                  (this->homing_current_ > 0 ? this->homing_current_ : this->settings_.run_current) * 1000,
                  this->stall_threshold_ * 100, this->max_travel_deg_, this->backoff_deg_);
  } else {
    ESP_LOGCONFIG(TAG, "  Calibration: off");
  }
  LOG_BINARY_SENSOR("  ", "Open limit", this->open_limit_);
  LOG_BINARY_SENSOR("  ", "Closed limit", this->closed_limit_);
  LOG_SENSOR("  ", "StallGuard", this->stallguard_sensor_);
  LOG_SENSOR("  ", "Travel", this->travel_sensor_);
  LOG_TEXT_SENSOR("  ", "Status", this->status_sensor_);
}

valve::ValveTraits ValveActuator::get_traits() {
  valve::ValveTraits traits;
  traits.set_supports_position(true);
  traits.set_supports_stop(true);
  return traits;
}

void ValveActuator::control(const valve::ValveCall &call) {
  bool calibrating = this->calibration_requested_ || this->homing_ != Homing::IDLE;
  if (call.get_stop()) {
    if (calibrating) {
      ESP_LOGI(TAG, "Calibration cancelled");
      this->calibration_requested_ = false;
      this->abort_calibration_();
    }
    this->stop_();
    return;
  }
  auto position = call.get_position();
  if (!position.has_value())
    return;
  float fraction = std::clamp(*position, 0.0f, 1.0f);
  if (calibrating) {
    ESP_LOGD(TAG, "Moving to %.0f%% once calibrated", fraction * 100);
    this->resume_position_ = fraction;
    return;
  }
  this->move_to_(static_cast<int32_t>(lroundf(fraction * this->travel_steps_)));
}

void ValveActuator::calibrate() {
  if (!this->homing_enabled_) {
    ESP_LOGW(TAG, "Calibration needs a homing: block in the valve config");
    return;
  }
  if (this->calibration_requested_ || this->homing_ != Homing::IDLE)
    return;
  ESP_LOGI(TAG, "Calibration requested");
  this->calibration_requested_ = true;
  // Come back to where the valve is headed.
  this->resume_position_ = std::clamp(float(this->target_) / this->travel_steps_, 0.0f, 1.0f);
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
    this->set_status_(this->calibration_result_);
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
    this->driver_lost_();
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

void ValveActuator::driver_lost_() {
  ESP_LOGW(TAG, "TMC2209 stopped responding (motor supply off?)");
  this->driver_ok_ = false;
  this->enable_pin_->digital_write(false);
  this->status_set_warning();
  this->set_status_(STATUS_NO_DRIVER);
  if (this->homing_ != Homing::IDLE) {
    // configure() restores the normal settings on reconnect; then start over.
    this->homing_ = Homing::IDLE;
    this->calibration_requested_ = true;
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
  this->start_move_(target, this->max_speed_);
}

void ValveActuator::start_move_(int32_t target, float speed_limit) {
  int8_t direction = target >= this->position_ ? 1 : -1;
  if (this->moving_ && direction != this->direction_)
    this->speed_ = 0;  // reverse from standstill
  this->target_ = target;
  this->speed_limit_ = speed_limit;
  this->end_found_ = false;
  if (!this->moving_) {
    this->enable_pin_->digital_write(true);
    this->moving_ = true;
    this->speed_ = 0;
    this->last_update_us_ = this->last_step_us_ = micros();
    this->high_freq_.start();
  }

  // SG_RESULT only reads in StealthChop, which calibration always uses.
  this->sg_active_ = this->homing_ != Homing::IDLE || !this->settings_.spread_cycle;
  if (this->sg_active_) {
    uint32_t full_step_us = static_cast<uint32_t>(1e6f * this->settings_.microsteps / speed_limit);
    this->sg_interval_us_ = std::max(SG_MIN_INTERVAL_US, full_step_us);
    this->sg_failures_ = 0;
    this->sg_samples_ = 0;
    this->sg_low_count_ = 0;
    this->sg_sum_ = 0;
    this->sg_baseline_ = 0;
    this->sg_min_ = UINT16_MAX;
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
    this->end_move_();
    return;
  }
  int8_t direction = this->target_ > this->position_ ? 1 : -1;
  if (this->limit_tripped_(direction)) {
    if (this->seeking_()) {
      ESP_LOGI(TAG, "%s limit switch reached", direction > 0 ? "Open" : "Closed");
    } else {
      ESP_LOGW(TAG, "%s limit tripped, stopping", direction > 0 ? "Open" : "Closed");
    }
    this->end_found_ = true;
    this->target_ = this->position_;
    this->end_move_();
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
  this->speed_ = std::min(std::max(this->speed_, this->min_speed_), this->speed_limit_);

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

void ValveActuator::end_move_() {
  this->moving_ = false;
  this->speed_ = 0;
  this->sg_active_ = false;
  this->high_freq_.stop();
  if (this->homing_ != Homing::IDLE) {
    this->calibration_step_();
    return;
  }
  this->go_idle_();
}

void ValveActuator::go_idle_() {
  if (!this->hold_when_idle_ || !this->driver_ok_)
    this->enable_pin_->digital_write(false);
  this->current_operation = valve::VALVE_OPERATION_IDLE;
  this->publish_position_(true);
  ESP_LOGD(TAG, "Stopped at %.1f deg", this->position_ / this->steps_per_deg_);
  if (this->driver_ok_)
    this->log_driver_status_();
}

void ValveActuator::publish_position_(bool save) {
  this->position = std::clamp(float(this->position_) / this->travel_steps_, 0.0f, 1.0f);
  this->publish_state(save);
  this->last_publish_ms_ = millis();
}

// --- Calibration ---------------------------------------------------------------

void ValveActuator::start_calibration_() {
  this->calibration_requested_ = false;
  this->homing_speed_ = this->homing_speed_deg_s_ * this->steps_per_deg_;
  float current = this->homing_current_ > 0 ? this->homing_current_ : this->settings_.run_current;
  ESP_LOGI(TAG, "Calibrating at %.1f deg/s and %.0f mA, stall below %.0f%% of free-running StallGuard",
           this->homing_speed_deg_s_, current * 1000, this->stall_threshold_ * 100);
  // StallGuard only works in StealthChop. Hold at full current as well, so
  // StealthChop tunes itself at the current it will run at.
  this->driver_.set_spread_cycle(false);
  this->driver_.set_current(this->settings_.sense_resistor, current, 1.0f);
  this->enable_pin_->digital_write(true);
  this->homing_ = Homing::SETTLE;
  this->settle_start_ms_ = millis();
  this->set_status_(STATUS_CALIBRATING);
}

// Moves calibration on to its next phase when the current one ends.
void ValveActuator::calibration_step_() {
  switch (this->homing_) {
    case Homing::SETTLE: {
      int32_t runup = static_cast<int32_t>(lroundf(RUNUP_DEG * this->steps_per_deg_));
      if (this->position_ < runup) {
        this->homing_ = Homing::RUNUP;
        this->start_move_(this->position_ + runup, this->homing_speed_);
      } else {
        this->start_seek_(-1);
      }
      return;
    }

    case Homing::RUNUP:
      this->start_seek_(-1);
      return;

    case Homing::SEEK_CLOSED:
      if (this->end_found_) {
        this->calibration_result_ = STATUS_READY;
      } else {
        ESP_LOGW(TAG,
                 "No stall within %.0f deg (free-running StallGuard %.0f, lowest %u); "
                 "assuming the valve is at its closed end",
                 this->max_travel_deg_, this->sg_baseline_, this->sg_min_);
        this->calibration_result_ = STATUS_NO_CLOSED_END;
      }
      // This is position 0 from now on.
      this->position_ = this->target_ = 0;
      this->calibrated_ = true;
      this->homing_ = Homing::BACKOFF;
      this->start_move_(static_cast<int32_t>(lroundf(this->backoff_deg_ * this->steps_per_deg_)),
                        this->homing_speed_);
      return;

    case Homing::BACKOFF:
      if (this->travel_configured_) {
        this->finish_calibration_();
      } else {
        this->start_seek_(1);
      }
      return;

    case Homing::SEEK_OPEN:
      if (this->end_found_) {
        this->set_travel_steps_(this->position_);
        float motor_deg = this->travel_steps_ * 360.0f / (this->steps_per_revolution_ * this->settings_.microsteps);
        this->travel_pref_.save(&motor_deg);
        ESP_LOGI(TAG, "Valve travel measured: %.1f deg", this->travel_deg_);
      } else {
        ESP_LOGW(TAG,
                 "No stall within %.0f deg of the closed end (free-running StallGuard %.0f, lowest %u); "
                 "travel stays at %.1f deg",
                 this->max_travel_deg_, this->sg_baseline_, this->sg_min_, this->travel_deg_);
        if (this->calibration_result_ == STATUS_READY)
          this->calibration_result_ = STATUS_NO_OPEN_END;
      }
      this->finish_calibration_();
      return;

    case Homing::IDLE:
      return;
  }
}

void ValveActuator::start_seek_(int8_t direction) {
  this->homing_ = direction < 0 ? Homing::SEEK_CLOSED : Homing::SEEK_OPEN;
  ESP_LOGI(TAG, "Looking for the %s end", direction < 0 ? "closed" : "open");
  int32_t distance = static_cast<int32_t>(lroundf(this->max_travel_deg_ * this->steps_per_deg_));
  this->start_move_(this->position_ + direction * distance, this->homing_speed_);
}

void ValveActuator::finish_calibration_() {
  this->homing_ = Homing::IDLE;
  this->restore_motion_settings_();
  if (this->calibration_result_ == STATUS_READY) {
    this->status_clear_warning();
  } else {
    this->status_set_warning();
  }
  this->set_status_(this->calibration_result_);

  float fraction = this->resume_position_.value_or(0.0f);
  this->resume_position_.reset();
  ESP_LOGI(TAG, "Calibration done, moving to %.0f%%", fraction * 100);
  this->go_idle_();
  this->move_to_(static_cast<int32_t>(lroundf(fraction * this->travel_steps_)));
}

void ValveActuator::abort_calibration_() {
  if (this->homing_ == Homing::IDLE)
    return;
  bool settling = this->homing_ == Homing::SETTLE;
  this->homing_ = Homing::IDLE;
  this->resume_position_.reset();
  this->restore_motion_settings_();
  this->set_status_(this->calibration_result_);
  // A move in progress decelerates (stop_()), then end_move_() goes idle.
  if (settling)
    this->go_idle_();
}

void ValveActuator::restore_motion_settings_() {
  if (!this->driver_ok_)
    return;
  this->driver_.set_spread_cycle(this->settings_.spread_cycle);
  this->driver_.set_current(this->settings_.sense_resistor, this->settings_.run_current,
                            this->settings_.hold_fraction);
}

// --- StallGuard ----------------------------------------------------------------

// Runs on every loop() while moving in StealthChop: collects the last SG_RESULT
// reply and sends the next request, without blocking the step timing.
void ValveActuator::poll_stallguard_() {
  uint32_t value;
  switch (this->driver_.poll_reply(&value)) {
    case TMC2209::Reply::WAITING:
      return;
    case TMC2209::Reply::OK:
      this->sg_failures_ = 0;
      this->on_stallguard_(value & 0x3FF);
      break;
    case TMC2209::Reply::FAILED:
      if (++this->sg_failures_ >= SG_MAX_FAILURES) {
        this->target_ = this->position_;  // steps aren't reaching a powered motor
        this->sg_active_ = false;
        this->driver_lost_();
        return;
      }
      break;
    case TMC2209::Reply::NONE:
      break;
  }
  uint32_t now = micros();
  if (now - this->last_sg_request_us_ >= this->sg_interval_us_) {
    this->last_sg_request_us_ = now;
    this->driver_.request(TMC2209::REG_SG_RESULT);
  }
}

void ValveActuator::on_stallguard_(uint16_t value) {
  uint32_t now = millis();
  if (now - this->last_sg_publish_ms_ >= SG_REPORT_MS) {
    this->last_sg_publish_ms_ = now;
    if (this->stallguard_sensor_ != nullptr)
      this->stallguard_sensor_->publish_state(value);
    if (this->homing_ != Homing::IDLE) {
      ESP_LOGD(TAG, "StallGuard %u at %.1f deg", value, this->position_ / this->steps_per_deg_);
    } else {
      ESP_LOGV(TAG, "StallGuard %u at %.1f deg", value, this->position_ / this->steps_per_deg_);
    }
  }
  if (!this->seeking_())
    return;
  // SG_RESULT only means something at a steady speed.
  if (this->speed_ < this->speed_limit_ * 0.99f)
    return;

  if (this->sg_samples_ < SG_SKIP_SAMPLES + SG_BASELINE_SAMPLES) {
    if (this->sg_samples_ >= SG_SKIP_SAMPLES)
      this->sg_sum_ += value;
    if (++this->sg_samples_ == SG_SKIP_SAMPLES + SG_BASELINE_SAMPLES) {
      this->sg_baseline_ = float(this->sg_sum_) / SG_BASELINE_SAMPLES;
      ESP_LOGI(TAG, "Free-running StallGuard %.0f, a stall is below %.0f", this->sg_baseline_,
               this->sg_baseline_ * this->stall_threshold_);
      if (this->sg_baseline_ < SG_USABLE_BASELINE) {
        ESP_LOGW(TAG, "StallGuard is near 0 while running free, so stalls can't be told apart; "
                      "try a faster calibration speed or a lower current");
      }
    }
    return;
  }

  this->sg_min_ = std::min(this->sg_min_, value);
  if (value >= this->sg_baseline_ * this->stall_threshold_) {
    this->sg_low_count_ = 0;
    return;
  }
  if (++this->sg_low_count_ < SG_STALL_SAMPLES)
    return;
  ESP_LOGI(TAG, "Stall at %.1f deg (StallGuard %u, free running %.0f)", this->position_ / this->steps_per_deg_,
           value, this->sg_baseline_);
  this->end_found_ = true;
  this->target_ = this->position_;  // stop here: this is the end stop
}

// --- State reporting -----------------------------------------------------------

void ValveActuator::set_status_(const char *status) {
  if (status == this->status_)
    return;
  this->status_ = status;
  if (this->status_sensor_ != nullptr)
    this->status_sensor_->publish_state(status);
}

void ValveActuator::set_travel_steps_(int32_t steps) {
  this->travel_steps_ = std::max<int32_t>(1, steps);
  this->travel_deg_ = this->travel_steps_ / this->steps_per_deg_;
  if (this->travel_sensor_ != nullptr)
    this->travel_sensor_->publish_state(this->travel_deg_);
}

}  // namespace esphome::valve_actuator
