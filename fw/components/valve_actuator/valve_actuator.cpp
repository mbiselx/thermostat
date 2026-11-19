#include "valve_actuator.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "esphome/core/log.h"

#ifdef USE_ESP32
#include <esp_system.h>
#endif

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
// so the seek has room to get up to speed before it reaches the stop.
static constexpr float RUNUP_DEG = 15.0f;
// A calibration that stalls further than this from where the closed end was
// expected hit something else (or the current is too low to turn the valve).
static constexpr float EARLY_STALL_DEG = 20.0f;

// StallGuard: SG_RESULT is updated once per full step, so poll about that
// often, but no faster than this.
static constexpr uint32_t SG_MIN_INTERVAL_US = 10000;
static constexpr uint32_t SG_REPORT_MS = 250;
static constexpr uint8_t SG_SKIP_SAMPLES = 4;      // readings ignored once at speed
static constexpr uint8_t SG_BASELINE_SAMPLES = 8;  // averaged into the free-running value
static constexpr uint8_t SG_STALL_SAMPLES = 3;     // consecutive low readings that make a stall
static constexpr uint8_t SG_MAX_FAILURES = 10;     // missed replies before the driver counts as gone
static constexpr float SG_USABLE_BASELINE = 10.0f;

// Encoder.
static constexpr uint32_t ENCODER_MOVING_US = 4000;  // read interval while stepping
static constexpr uint32_t ENCODER_IDLE_US = 10000;   // and while idle (hand turning)
static constexpr uint8_t ENCODER_MAX_FAILURES = 20;
static constexpr uint8_t LAG_BASELINE_SAMPLES = 8;
static constexpr int32_t STALL_LAG_FULL_STEPS = 3;  // lagging this much more than at the start = stalled
static constexpr int32_t LOST_STEP_FULL_STEPS = 4;  // move-end mismatch worth correcting
static constexpr int32_t HAND_TURN_FULL_STEPS = 2;  // smaller changes at idle are detent settling
static constexpr uint32_t HAND_TURN_SETTLE_MS = 2000;
static constexpr int32_t ENCODER_COUNTS = 16384;  // MT6816 counts per motor turn
// An unpowered rotor can settle a full step either way, so allow a bit more.
static constexpr int32_t RESTORE_TOLERANCE_COUNTS = ENCODER_COUNTS * 3 / 200;  // 3 full steps

// Preferences. New keys for this layout, so values saved by older firmware are
// ignored rather than misread.
static constexpr uint32_t TRAVEL_PREF_VERSION = 0x54524132;  // "TRA2": travel, motor degrees
static constexpr uint32_t STATE_PREF_VERSION = 0x53544131;   // "STA1": SavedState
static constexpr uint8_t STATE_CALIBRATED = 1 << 0;
static constexpr uint8_t STATE_ALERT = 1 << 1;
static constexpr uint8_t STATE_ENCODER = 1 << 2;  // encoder_angle is meaningful

static const char *const STATUS_NO_DRIVER = "No motor supply";
static const char *const STATUS_CALIBRATING = "Calibrating";
static const char *const STATUS_RECOVERING = "Recovering position";
static const char *const STATUS_READY = "Ready";
static const char *const STATUS_END_ASSUMED = "Ready, end not detected";
static const char *const STATUS_NOT_CALIBRATED = "Not calibrated";
static const char *const STATUS_EARLY_STALL = "Stalled before the end";
static const char *const STATUS_NO_OPEN_END = "Open end not found";

static int32_t wrap_counts(int32_t delta) {
  if (delta >= ENCODER_COUNTS / 2)
    return delta - ENCODER_COUNTS;
  if (delta < -ENCODER_COUNTS / 2)
    return delta + ENCODER_COUNTS;
  return delta;
}

static bool software_restart() {
#ifdef USE_ESP32
  esp_reset_reason_t reason = esp_reset_reason();
  return reason == ESP_RST_SW || reason == ESP_RST_DEEPSLEEP;
#else
  return false;
#endif
}

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

  this->steps_per_turn_ = this->steps_per_revolution_ * this->settings_.microsteps;
  this->steps_per_deg_ = this->steps_per_turn_ * this->gear_ratio_ / 360.0f;
  this->max_speed_ = this->max_speed_deg_s_ * this->steps_per_deg_;
  this->accel_ = this->accel_deg_s2_ * this->steps_per_deg_;
  this->min_speed_ = sqrtf(2.0f * this->accel_);  // speed after one step from rest
  this->calibration_result_ = STATUS_NOT_CALIBRATED;

  // Travel: as set by hand or measured, else as configured, else the search
  // limit until it's known.
  this->travel_pref_ = this->make_entity_preference<float>(TRAVEL_PREF_VERSION);
  float motor_deg;
  if (this->travel_pref_.load(&motor_deg) && motor_deg > 0) {
    this->travel_deg_ = motor_deg / this->gear_ratio_;
    this->travel_known_ = true;
  } else if (this->travel_configured_) {
    this->travel_known_ = true;
  } else {
    this->travel_deg_ = this->max_travel_deg_;
    ESP_LOGW(TAG, "Valve travel not known yet; calibrate, or use Set open");
  }
  this->travel_steps_ = std::max<int32_t>(1, static_cast<int32_t>(lroundf(this->travel_deg_ * this->steps_per_deg_)));
  if (this->travel_known_ && this->travel_sensor_ != nullptr)
    this->travel_sensor_->publish_state(this->travel_deg_);

  // Position: trust the saved one if the encoder shows the motor hasn't
  // turned since (or, without an encoder, after a software restart).
#ifdef USE_SPI
  if (this->encoder_ != nullptr)
    this->encoder_->setup();
#endif
  uint16_t angle = 0;
  bool angle_ok = this->read_encoder_(&angle);
  if (angle_ok) {
    this->encoder_angle_ = angle;
    this->encoder_ok_ = true;
  }

  this->state_pref_ = this->make_entity_preference<SavedState>(STATE_PREF_VERSION);
  SavedState saved{};
  bool trusted = false;
  if (this->state_pref_.load(&saved)) {
    this->position_ = saved.position;
    this->calibrated_ = saved.flags & STATE_CALIBRATED;
    this->alert_ = saved.flags & STATE_ALERT;
    if (angle_ok && (saved.flags & STATE_ENCODER) && saved.encoder_sign != 0) {
      this->encoder_sign_ = saved.encoder_sign;
      int32_t moved = wrap_counts(int32_t(angle) - int32_t(saved.encoder_angle));
      if (std::abs(moved) <= RESTORE_TOLERANCE_COUNTS) {
        this->position_ += static_cast<int32_t>(int64_t(moved) * this->steps_per_turn_ * this->encoder_sign_ /
                                                ENCODER_COUNTS);
        trusted = true;
        ESP_LOGI(TAG, "Encoder confirms the saved position");
      } else {
        ESP_LOGW(TAG, "Motor turned %.0f deg since the position was saved", moved * 360.0f / ENCODER_COUNTS);
      }
    } else if (software_restart()) {
      trusted = true;
      ESP_LOGI(TAG, "Software restart, keeping the saved position");
    }
  } else {
    // First boot with this firmware: take the valve's own saved position.
    auto restored = this->restore_state_();
    if (restored.has_value()) {
      this->position_ = static_cast<int32_t>(lroundf(restored->position * this->travel_steps_));
      trusted = software_restart();
    }
  }
  this->target_ = this->position_;
  this->sync_encoder_();
  if (trusted && this->calibrated_)
    this->calibration_result_ = STATUS_READY;
  if (this->alert_sensor_ != nullptr)
    this->alert_sensor_->publish_state(this->alert_);

  this->position = std::clamp(float(this->position_) / this->travel_steps_, 0.0f, 1.0f);
  this->current_operation = valve::VALVE_OPERATION_IDLE;
  this->publish_state(false);

  if (!trusted && this->homing_enabled_ && this->home_on_boot_) {
    // Runs from loop() once every component is set up (template numbers may
    // change the speed, current or threshold during their setup).
    ESP_LOGW(TAG, "Position not trusted after this reset; recovering it");
    this->calibration_requested_ = true;
    this->recovering_ = true;
    this->resume_position_ = this->recovery_position_;
  }

  this->driver_.set_uart(this);
  if (!this->connect_driver_()) {
    ESP_LOGW(TAG, "No TMC2209 answering yet, check the 9-12 V motor supply");
    this->status_set_warning();
    this->set_status_(STATUS_NO_DRIVER);
  }
}

void ValveActuator::loop() {
  this->poll_encoder_();
  if (this->moving_) {
    this->run_motion_();
    if (this->moving_ && this->sg_active_)
      this->poll_stallguard_();
    return;
  }
  this->track_manual_moves_();
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
                "  Chopper: %s, %s when idle\n"
                "  Encoder: %s",
                this->driver_ok_ ? "connected" : "not found", this->driver_.get_address(),
                this->settings_.run_current * 1000, this->driver_.get_actual_current() * 1000,
                this->settings_.hold_fraction * 100, this->settings_.microsteps, this->gear_ratio_,
                this->steps_per_deg_, this->travel_deg_, (int) this->travel_steps_,
                this->travel_known_ ? "known" : "not known yet", this->max_speed_deg_s_, this->accel_deg_s2_,
                this->settings_.spread_cycle ? "SpreadCycle" : "StealthChop",
                this->hold_when_idle_ ? "holding" : "unpowered",
                !this->encoder_ok_ ? "not available" : (this->encoder_sign_ != 0 ? "tracking" : "direction not learned yet"));
  if (this->homing_enabled_) {
    ESP_LOGCONFIG(TAG,
                  "  Calibration: %.1f deg/s, %.0f mA, searching up to %.0f deg, back-off %.1f deg\n"
                  "  After a power cut: %s, then %.0f%%\n"
                  "  StallGuard stall: below %.0f%% of free running",
                  this->homing_speed_deg_s_,
                  (this->homing_current_ > 0 ? this->homing_current_ : this->settings_.run_current) * 1000,
                  this->max_travel_deg_, this->backoff_deg_,
                  this->home_on_boot_ ? "find the closed end" : "nothing", this->recovery_position_ * 100,
                  this->stall_threshold_ * 100);
  } else {
    ESP_LOGCONFIG(TAG, "  Calibration: off");
  }
  LOG_BINARY_SENSOR("  ", "Open limit", this->open_limit_);
  LOG_BINARY_SENSOR("  ", "Closed limit", this->closed_limit_);
  LOG_SENSOR("  ", "StallGuard", this->stallguard_sensor_);
  LOG_SENSOR("  ", "Travel", this->travel_sensor_);
  LOG_TEXT_SENSOR("  ", "Status", this->status_sensor_);
  LOG_BINARY_SENSOR("  ", "Alert", this->alert_sensor_);
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

// --- Buttons -------------------------------------------------------------------

void ValveActuator::calibrate() {
  if (!this->homing_enabled_) {
    ESP_LOGW(TAG, "Calibration needs a homing: block in the valve config");
    return;
  }
  if (this->calibration_requested_ || this->homing_ != Homing::IDLE)
    return;
  ESP_LOGI(TAG, "Calibration requested");
  this->calibration_requested_ = true;
  this->recovering_ = false;
  // Come back to where the valve is headed.
  this->resume_position_ = std::clamp(float(this->target_) / this->travel_steps_, 0.0f, 1.0f);
}

void ValveActuator::set_closed_here() {
  if (this->moving_ || this->homing_ != Homing::IDLE) {
    ESP_LOGW(TAG, "Not setting the closed end while the valve is moving");
    return;
  }
  ESP_LOGI(TAG, "Closed end set here (%.1f deg from the old one)", this->position_ / this->steps_per_deg_);
  this->calibration_requested_ = false;
  this->recovering_ = false;
  this->position_ = this->target_ = 0;
  this->sync_encoder_();
  this->calibrated_ = true;
  this->calibration_result_ = STATUS_READY;
  if (this->driver_ok_) {
    this->status_clear_warning();
    this->set_status_(STATUS_READY);
  }
  this->set_alert_(false);
  this->publish_position_(true);
  this->save_state_();
}

void ValveActuator::set_open_here() {
  if (this->moving_ || this->homing_ != Homing::IDLE) {
    ESP_LOGW(TAG, "Not setting the open end while the valve is moving");
    return;
  }
  if (this->position_ < this->steps_per_deg_ * 10) {
    ESP_LOGW(TAG, "Not setting the open end only %.1f deg from the closed end; set the closed end first",
             this->position_ / this->steps_per_deg_);
    return;
  }
  this->set_travel_steps_(this->position_);
  this->travel_known_ = true;
  this->save_travel_();
  ESP_LOGI(TAG, "Open end set here: travel is %.1f deg", this->travel_deg_);
  this->target_ = this->position_;
  this->publish_position_(true);
  this->save_state_();
}

void ValveActuator::jog(float degrees) {
  if (this->homing_ != Homing::IDLE || this->calibration_requested_) {
    ESP_LOGW(TAG, "Not jogging during calibration");
    return;
  }
  int32_t from = this->moving_ ? this->target_ : this->position_;
  this->move_to_(from + static_cast<int32_t>(lroundf(degrees * this->steps_per_deg_)));
}

void ValveActuator::dismiss_alert() {
  this->set_alert_(false);
  this->save_state_();
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
  this->turned_by_hand_ = false;
  if (!this->moving_) {
    this->enable_pin_->digital_write(true);
    this->moving_ = true;
    this->speed_ = 0;
    this->last_update_us_ = this->last_step_us_ = micros();
    this->move_start_position_ = this->position_;
    this->move_start_counts_ = this->encoder_counts_;
    this->high_freq_.start();
  }
  this->lag_samples_ = 0;
  this->lag_sum_ = 0;
  this->lag_baseline_ = 0;
  this->lag_max_ = 0;

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
      this->end_found_at_("limit switch");
    } else {
      ESP_LOGW(TAG, "%s limit tripped, stopping", direction > 0 ? "Open" : "Closed");
      this->target_ = this->position_;
    }
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
  this->check_encoder_after_move_();
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
  this->save_state_();
  ESP_LOGD(TAG, "Stopped at %.1f deg", this->position_ / this->steps_per_deg_);
  if (this->driver_ok_)
    this->log_driver_status_();
}

void ValveActuator::publish_position_(bool save) {
  this->position = std::clamp(float(this->position_) / this->travel_steps_, 0.0f, 1.0f);
  this->publish_state(save);
  this->last_publish_ms_ = millis();
}

void ValveActuator::save_state_() {
  SavedState state{};
  state.position = this->position_;
  if (this->tracking_()) {
    state.encoder_angle = this->encoder_angle_;
    state.encoder_sign = this->encoder_sign_;
    state.flags |= STATE_ENCODER;
  }
  if (this->calibrated_)
    state.flags |= STATE_CALIBRATED;
  if (this->alert_)
    state.flags |= STATE_ALERT;
  this->state_pref_.save(&state);
}

// --- Calibration ---------------------------------------------------------------

void ValveActuator::start_calibration_() {
  this->calibration_requested_ = false;
  this->homing_speed_ = this->homing_speed_deg_s_ * this->steps_per_deg_;
  float current = this->homing_current_ > 0 ? this->homing_current_ : this->settings_.run_current;
  ESP_LOGI(TAG, "%s at %.1f deg/s and %.0f mA", this->recovering_ ? "Recovering the position" : "Calibrating",
           this->homing_speed_deg_s_, current * 1000);
  // StallGuard only works in StealthChop. Hold at the same current as well,
  // so StealthChop tunes itself at the current it will run at.
  this->driver_.set_spread_cycle(false);
  this->driver_.set_current(this->settings_.sense_resistor, current, 1.0f);
  this->enable_pin_->digital_write(true);
  this->homing_ = Homing::SETTLE;
  this->settle_start_ms_ = millis();
  this->set_status_(this->recovering_ ? STATUS_RECOVERING : STATUS_CALIBRATING);
}

// Moves calibration on to its next phase when the current one ends.
void ValveActuator::calibration_step_() {
  switch (this->homing_) {
    case Homing::SETTLE: {
      // A short move first: away from the closed end if it's close, so the
      // seek can get up to speed (stalls only show at speed), and to learn the
      // encoder's direction if it isn't known yet (towards the middle, to stay
      // clear of either end). When recovering, the saved position is only a
      // guess, but still the best one.
      int32_t runup = static_cast<int32_t>(lroundf(RUNUP_DEG * this->steps_per_deg_));
      bool near_closed = this->position_ < runup;
      bool learn_encoder = this->encoder_ok_ && this->encoder_sign_ == 0;
      if (near_closed || learn_encoder) {
        int8_t direction = this->position_ < this->travel_steps_ / 2 ? 1 : -1;
        this->homing_ = Homing::RUNUP;
        this->start_move_(this->position_ + direction * runup, this->homing_speed_);
      } else {
        this->start_seek_(-1);
      }
      return;
    }

    case Homing::RUNUP:
      this->start_seek_(-1);
      return;

    case Homing::SEEK_CLOSED: {
      float off_by = this->position_ / this->steps_per_deg_;
      if (this->end_found_ && !this->recovering_ && this->calibrated_ && off_by > EARLY_STALL_DEG) {
        // Stopped well before where the closed end was: keep the old zero.
        ESP_LOGW(TAG, "Stalled %.0f deg before the closed end; keeping the old position. If nothing is in the way, "
                      "raise the calibration current",
                 off_by);
        this->calibration_result_ = STATUS_EARLY_STALL;
        this->finish_calibration_();
        return;
      }
      if (this->end_found_) {
        this->calibration_result_ = STATUS_READY;
      } else {
        ESP_LOGW(TAG, "No end detected within %.0f deg; assuming the valve is at its closed end", this->max_travel_deg_);
        this->calibration_result_ = STATUS_END_ASSUMED;
      }
      // This is position 0 from now on.
      this->position_ = this->target_ = 0;
      this->sync_encoder_();
      this->calibrated_ = true;
      this->homing_ = Homing::BACKOFF;
      this->start_move_(static_cast<int32_t>(lroundf(this->backoff_deg_ * this->steps_per_deg_)),
                        this->homing_speed_);
      return;
    }

    case Homing::BACKOFF:
      if (this->recovering_ || this->travel_known_) {
        this->finish_calibration_();
      } else {
        this->start_seek_(1);
      }
      return;

    case Homing::SEEK_OPEN:
      if (this->end_found_) {
        this->set_travel_steps_(this->position_);
        this->travel_known_ = true;
        this->save_travel_();
        ESP_LOGI(TAG, "Valve travel measured: %.1f deg", this->travel_deg_);
      } else {
        ESP_LOGW(TAG, "No open end detected within %.0f deg; travel stays at %.1f deg", this->max_travel_deg_,
                 this->travel_deg_);
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

// The seek reached an end stop: stop here.
void ValveActuator::end_found_at_(const char *how) {
  ESP_LOGI(TAG, "End found at %.1f deg (%s)", this->position_ / this->steps_per_deg_, how);
  this->end_found_ = true;
  this->target_ = this->position_;
}

void ValveActuator::finish_calibration_() {
  bool recovered = this->recovering_;
  this->homing_ = Homing::IDLE;
  this->recovering_ = false;
  this->restore_motion_settings_();
  if (this->calibration_result_ == STATUS_READY) {
    this->status_clear_warning();
  } else {
    this->status_set_warning();
  }
  this->set_status_(this->calibration_result_);
  // After a power cut someone should check on the valve; a calibration they
  // asked for clears that.
  this->set_alert_(recovered);

  float fraction = this->resume_position_.value_or(0.0f);
  this->resume_position_.reset();
  ESP_LOGI(TAG, "%s, moving to %.0f%%", recovered ? "Position recovered" : "Calibration done", fraction * 100);
  this->go_idle_();
  this->move_to_(static_cast<int32_t>(lroundf(fraction * this->travel_steps_)));
}

void ValveActuator::abort_calibration_() {
  if (this->homing_ == Homing::IDLE)
    return;
  bool settling = this->homing_ == Homing::SETTLE;
  this->homing_ = Homing::IDLE;
  this->recovering_ = false;
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
  this->sg_last_ = value;
  uint32_t now = millis();
  if (now - this->last_sg_publish_ms_ >= SG_REPORT_MS) {
    this->last_sg_publish_ms_ = now;
    if (this->stallguard_sensor_ != nullptr)
      this->stallguard_sensor_->publish_state(value);
    int32_t lag = this->tracking_() ? (this->position_ - this->encoder_position_()) * this->direction_ : 0;
    if (this->homing_ != Homing::IDLE) {
      ESP_LOGD(TAG, "StallGuard %u, lag %.2f motor deg at %.1f deg", value, lag * 360.0f / this->steps_per_turn_,
               this->position_ / this->steps_per_deg_);
    } else {
      ESP_LOGV(TAG, "StallGuard %u, lag %.2f motor deg at %.1f deg", value, lag * 360.0f / this->steps_per_turn_,
               this->position_ / this->steps_per_deg_);
    }
  }
  if (!this->seeking_() || this->end_found_)
    return;
  // SG_RESULT only means something at a steady speed.
  if (this->speed_ < this->speed_limit_ * 0.99f)
    return;

  if (this->sg_samples_ < SG_SKIP_SAMPLES + SG_BASELINE_SAMPLES) {
    if (this->sg_samples_ >= SG_SKIP_SAMPLES)
      this->sg_sum_ += value;
    if (++this->sg_samples_ == SG_SKIP_SAMPLES + SG_BASELINE_SAMPLES) {
      this->sg_baseline_ = float(this->sg_sum_) / SG_BASELINE_SAMPLES;
      ESP_LOGD(TAG, "Free-running StallGuard %.0f, a stall is below %.0f", this->sg_baseline_,
               this->sg_baseline_ * this->stall_threshold_);
      if (this->sg_baseline_ < SG_USABLE_BASELINE) {
        ESP_LOGW(TAG, "StallGuard is near 0 while running free, so stalls can't be told apart");
      }
    }
    return;
  }

  this->sg_min_ = std::min(this->sg_min_, value);
  if (value >= this->sg_baseline_ * this->stall_threshold_) {
    this->sg_low_count_ = 0;
    return;
  }
  if (++this->sg_low_count_ >= SG_STALL_SAMPLES)
    this->end_found_at_("StallGuard");
}

// --- Encoder -------------------------------------------------------------------

bool ValveActuator::read_encoder_(uint16_t *angle) {
#ifdef USE_SPI
  if (this->encoder_ == nullptr)
    return false;
  switch (this->encoder_->read(angle)) {
    case MT6816::Result::OK:
      this->encoder_failures_ = 0;
      return true;
    case MT6816::Result::NO_MAGNET:
      if (!this->encoder_warned_) {
        ESP_LOGW(TAG, "Encoder sees no magnet on the motor shaft; position comes from counting steps only");
        this->encoder_warned_ = true;
      }
      this->encoder_ok_ = false;
      return false;
    case MT6816::Result::BAD_PARITY:
      if (++this->encoder_failures_ >= ENCODER_MAX_FAILURES && this->encoder_ok_) {
        ESP_LOGW(TAG, "Encoder readings keep failing; position comes from counting steps only");
        this->encoder_ok_ = false;
      }
      return false;
  }
#endif
  return false;
}

// Reads the encoder now and then and accumulates whole turns.
void ValveActuator::poll_encoder_() {
  if (!this->encoder_ok_)
    return;
  uint32_t now = micros();
  if (now - this->last_encoder_read_us_ < (this->moving_ ? ENCODER_MOVING_US : ENCODER_IDLE_US))
    return;
  this->last_encoder_read_us_ = now;
  uint16_t angle;
  if (!this->read_encoder_(&angle))
    return;
  this->encoder_counts_ += wrap_counts(int32_t(angle) - int32_t(this->encoder_angle_));
  this->encoder_angle_ = angle;
  if (this->moving_)
    this->check_encoder_stall_();
}

int32_t ValveActuator::encoder_steps_() const {
  return static_cast<int32_t>(int64_t(this->encoder_counts_) * this->steps_per_turn_ * this->encoder_sign_ /
                              ENCODER_COUNTS);
}

int32_t ValveActuator::encoder_position_() const { return this->encoder_steps_() + this->encoder_offset_; }

void ValveActuator::sync_encoder_() { this->encoder_offset_ = this->position_ - this->encoder_steps_(); }

// During a seek: the motor has stalled if the shaft falls behind the steps
// sent, by more than the lag it ran with at the start.
void ValveActuator::check_encoder_stall_() {
  if (!this->seeking_() || this->end_found_ || !this->tracking_())
    return;
  if (this->speed_ < this->speed_limit_ * 0.99f)
    return;
  int32_t lag = (this->position_ - this->encoder_position_()) * this->direction_;
  if (this->lag_samples_ < LAG_BASELINE_SAMPLES) {
    this->lag_sum_ += lag;
    if (++this->lag_samples_ == LAG_BASELINE_SAMPLES)
      this->lag_baseline_ = this->lag_sum_ / LAG_BASELINE_SAMPLES;
    return;
  }
  int32_t extra = lag - this->lag_baseline_;
  this->lag_max_ = std::max(this->lag_max_, extra);
  if (extra > STALL_LAG_FULL_STEPS * this->settings_.microsteps)
    this->end_found_at_("motor stalled");
}

// After a move: learn which way the encoder counts, notice an encoder that
// isn't following the motor, and correct for lost steps.
void ValveActuator::check_encoder_after_move_() {
  if (!this->encoder_ok_)
    return;
  int32_t commanded = this->position_ - this->move_start_position_;
  int32_t counts = this->encoder_counts_ - this->move_start_counts_;
  int32_t measured = static_cast<int32_t>(int64_t(counts) * this->steps_per_turn_ / ENCODER_COUNTS);
  // Only moves that ran their full course say anything about the encoder: a
  // seek that found its end may have stalled the motor on purpose.
  bool ran_free = !(this->seeking_() && this->end_found_);
  if (ran_free && std::abs(commanded) >= this->steps_per_turn_ / 16) {
    float ratio = float(measured) / float(commanded);
    if (std::fabs(ratio) < 0.1f) {
      ESP_LOGW(TAG, "Encoder isn't following the motor (magnet missing or loose?); "
                    "position comes from counting steps only");
      this->encoder_ok_ = false;
      this->encoder_sign_ = 0;
      return;
    }
    int8_t sign = ratio > 0 ? 1 : -1;
    if (std::fabs(std::fabs(ratio) - 1.0f) < 0.25f && sign != this->encoder_sign_) {
      ESP_LOGI(TAG, "Encoder direction %s", this->encoder_sign_ == 0 ? "learned" : "changed");
      this->encoder_sign_ = sign;
      this->sync_encoder_();
      return;
    }
  }
  if (!this->tracking_())
    return;
  if (this->lag_max_ > 0) {
    ESP_LOGD(TAG, "Largest extra lag on that move: %.2f motor deg", this->lag_max_ * 360.0f / this->steps_per_turn_);
  }
  int32_t error = this->position_ - this->encoder_position_();
  if (std::abs(error) > LOST_STEP_FULL_STEPS * this->settings_.microsteps) {
    float lost_deg = std::abs(error) / this->steps_per_deg_;
    if (ran_free) {
      ESP_LOGW(TAG, "Motor lost %.1f deg of valve travel on that move; taking the encoder's position", lost_deg);
    } else {
      // A seek that stalled at the end stop: the steps sent after the stall
      // never turned the motor.
      ESP_LOGD(TAG, "Motor stopped %.1f deg short of the steps sent; taking the encoder's position", lost_deg);
    }
    this->position_ = this->target_ = this->encoder_position_();
  }
  this->sync_encoder_();
}

// With the motor unpowered, the valve can be turned by hand: follow it.
void ValveActuator::track_manual_moves_() {
  if (!this->tracking_() || this->homing_ != Homing::IDLE)
    return;
  int32_t measured = this->encoder_position_();
  uint32_t now = millis();
  if (std::abs(measured - this->position_) >= HAND_TURN_FULL_STEPS * this->settings_.microsteps) {
    if (!this->turned_by_hand_) {
      ESP_LOGD(TAG, "Valve is being turned by hand");
    }
    this->turned_by_hand_ = true;
    this->last_hand_turn_ms_ = now;
    this->position_ = this->target_ = measured;
    if (now - this->last_publish_ms_ >= PUBLISH_INTERVAL_MS)
      this->publish_position_(false);
  } else if (this->turned_by_hand_ && now - this->last_hand_turn_ms_ >= HAND_TURN_SETTLE_MS) {
    this->turned_by_hand_ = false;
    this->position_ = this->target_ = measured;
    ESP_LOGI(TAG, "Turned by hand to %.1f deg (%.0f%%)", this->position_ / this->steps_per_deg_,
             100.0f * this->position_ / this->travel_steps_);
    this->publish_position_(true);
    this->save_state_();
  }
}

// --- State reporting -----------------------------------------------------------

void ValveActuator::set_status_(const char *status) {
  if (status == this->status_)
    return;
  this->status_ = status;
  if (this->status_sensor_ != nullptr)
    this->status_sensor_->publish_state(status);
}

void ValveActuator::set_alert_(bool alert) {
  if (alert != this->alert_) {
    ESP_LOGI(TAG, "Maintenance alert %s", alert ? "raised" : "cleared");
  }
  this->alert_ = alert;
  if (this->alert_sensor_ != nullptr)
    this->alert_sensor_->publish_state(alert);
}

void ValveActuator::set_travel_steps_(int32_t steps) {
  this->travel_steps_ = std::max<int32_t>(1, steps);
  this->travel_deg_ = this->travel_steps_ / this->steps_per_deg_;
  if (this->travel_sensor_ != nullptr)
    this->travel_sensor_->publish_state(this->travel_deg_);
}

void ValveActuator::save_travel_() {
  float motor_deg = this->travel_steps_ * 360.0f / this->steps_per_turn_;
  this->travel_pref_.save(&motor_deg);
}

}  // namespace esphome::valve_actuator
