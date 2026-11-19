#include "radiator_climate.h"

#ifdef USE_CLIMATE

#include <algorithm>
#include <cmath>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#ifdef USE_API
#include "esphome/components/api/api_server.h"
#endif

namespace esphome::valve_actuator {

static const char *const TAG = "valve_actuator.climate";

static constexpr uint32_t INTEGRAL_PREF_VERSION = 0x50493031;  // "PI01"
// A long gap between updates (sensor offline, loop stalled) counts as this.
static constexpr float MAX_STEP_HOURS = 0.25f;
static constexpr float CLOSED = 0.005f;

void RadiatorClimate::setup() {
  this->integral_pref_ = this->make_entity_preference<float>(INTEGRAL_PREF_VERSION);
  float saved;
  if (this->integral_pref_.load(&saved) && std::isfinite(saved)) {
    this->integral_ = this->saved_integral_ = std::clamp(saved, 0.0f, this->max_position_);
  }

  this->sensor_->add_on_state_callback([this](float state) {
    bool first = std::isnan(this->current_temperature) && !std::isnan(state);
    this->current_temperature = state;
    if (first) {
      this->update();  // react to the first reading instead of waiting
    } else {
      this->publish_state();
    }
  });
  this->current_temperature = this->sensor_->state;
  if (this->humidity_sensor_ != nullptr) {
    this->humidity_sensor_->add_on_state_callback([this](float state) {
      this->current_humidity = state;
      this->publish_state();
    });
    this->current_humidity = this->humidity_sensor_->state;
  }

  this->valve_->add_on_manual_move_callback([this]() { this->set_held_(true); });

  this->restoring_ = true;
  auto restore = this->restore_state_();
  if (restore.has_value()) {
    restore->to_call(this).perform();
  } else {
    // First boot: heat, at the Home preset if there is one.
    this->mode = climate::CLIMATE_MODE_HEAT;
    this->target_temperature = this->default_target_;
    const Preset *home = this->find_preset_(climate::CLIMATE_PRESET_HOME);
    if (home != nullptr) {
      this->set_preset_(climate::CLIMATE_PRESET_HOME);
      this->target_temperature = home->target;
    } else if (!this->presets_.empty()) {
      this->set_preset_(climate::CLIMATE_PRESET_NONE);
    }
    this->publish_state();
  }
  this->restoring_ = false;
  if (this->held_sensor_ != nullptr)
    this->held_sensor_->publish_state(false);
}

void RadiatorClimate::control(const climate::ClimateCall &call) {
  if (call.get_mode().has_value())
    this->mode = *call.get_mode();

  const auto &preset = call.get_preset();
  const Preset *chosen = nullptr;
  if (preset.has_value() && !this->presets_.empty()) {
    this->set_preset_(*preset);
    chosen = this->find_preset_(*preset);
    if (chosen != nullptr) {
      this->target_temperature = chosen->target;
      // Start from where the valve usually sits for this preset, rather than
      // where it sat for the last one.
      if (!this->restoring_ && !std::isnan(chosen->valve_position))
        this->integral_ = std::min(chosen->valve_position, this->max_position_);
    }
  }
  const auto &target = call.get_target_temperature();
  // After a reboot, a preset's temperature comes from the config, not from
  // what was saved, in case the config changed.
  if (target.has_value() && !(this->restoring_ && chosen != nullptr)) {
    this->target_temperature = *target;
    // A temperature set by hand isn't any preset's.
    if (!preset.has_value() && !this->presets_.empty())
      this->set_preset_(climate::CLIMATE_PRESET_NONE);
  }

  if (this->restoring_) {
    this->publish_state();
    return;
  }
  // Any change to the thermostat takes the valve back from manual control.
  this->set_held_(false);
  if (this->mode == climate::CLIMATE_MODE_OFF)
    this->move_valve_(0.0f, true);
  this->publish_state();
  this->update();
}

void RadiatorClimate::update() {
  uint32_t now = millis();
  float hours = this->last_update_ms_ == 0 ? 0.0f : std::min((now - this->last_update_ms_) / 3.6e6f, MAX_STEP_HOURS);
  this->last_update_ms_ = now;

  if (this->mode == climate::CLIMATE_MODE_OFF) {
    if (!this->held_)
      this->move_valve_(0.0f, false);
    this->update_action_();
    return;
  }

  float temperature = this->current_temperature;
  bool stale = false;
#ifdef USE_API
  // A sensor from Home Assistant keeps its last value while disconnected.
  stale = api::global_api_server != nullptr && !api::global_api_server->is_connected();
#endif
  if (stale || std::isnan(temperature) || std::isnan(this->target_temperature)) {
    if (!this->warned_no_temperature_) {
      ESP_LOGW(TAG, "No room temperature; leaving the valve where it is");
      this->warned_no_temperature_ = true;
    }
    this->update_action_();
    return;
  }
  this->warned_no_temperature_ = false;

  float error = this->target_temperature - temperature;
  if (std::isnan(this->integral_)) {
    // Nothing learned yet: assume the valve's current position is about what
    // holds the target.
    this->integral_ = std::clamp(this->valve_->target_fraction(), 0.0f, this->max_position_);
  } else if (!this->held_ && std::fabs(error) > this->deadband_) {
    // Integrate only outside the deadband, and not while the valve is already
    // fully open (or closed) and the error asks for more of the same.
    float unclamped = this->kp_ * error + this->integral_;
    bool past_limit = (unclamped >= this->max_position_ && error > 0) || (unclamped <= 0.0f && error < 0);
    if (!past_limit)
      this->integral_ = std::clamp(this->integral_ + this->ki_ * error * hours, 0.0f, this->max_position_);
  }
  float output = std::clamp(this->kp_ * error + this->integral_, 0.0f, this->max_position_);
  ESP_LOGD(TAG, "Room %.1f, target %.1f: valve %.0f%% (P %+.0f%%, I %.0f%%)%s", temperature, this->target_temperature,
           output * 100, this->kp_ * error * 100, this->integral_ * 100, this->held_ ? ", held by hand" : "");

  if (!this->held_)
    this->move_valve_(output, false);
  this->save_integral_();
  this->update_action_();
}

// Moves the valve only if it makes a difference: at least min_step, or all the
// way closed.
void RadiatorClimate::move_valve_(float output, bool force) {
  float current = this->valve_->target_fraction();
  float change = std::fabs(output - current);
  if (change < 0.001f)
    return;
  bool closing = output <= 0.0f && current > 0.0f;
  if (!force && !closing && change < this->min_step_)
    return;
  ESP_LOGI(TAG, "Moving the valve from %.0f%% to %.0f%%", current * 100, output * 100);
  this->valve_->move_to_fraction(output);
}

void RadiatorClimate::set_held_(bool held) {
  if (held == this->held_)
    return;
  this->held_ = held;
  if (held) {
    ESP_LOGI(TAG, "Valve moved directly; holding it until the thermostat's mode, target or preset changes");
  } else {
    ESP_LOGI(TAG, "Thermostat back in control of the valve");
  }
  if (this->held_sensor_ != nullptr)
    this->held_sensor_->publish_state(held);
}

void RadiatorClimate::update_action_() {
  climate::ClimateAction action;
  if (this->mode == climate::CLIMATE_MODE_OFF) {
    action = climate::CLIMATE_ACTION_OFF;
  } else if (this->valve_->target_fraction() > CLOSED) {
    action = climate::CLIMATE_ACTION_HEATING;
  } else {
    action = climate::CLIMATE_ACTION_IDLE;
  }
  if (action != this->action) {
    this->action = action;
    this->publish_state();
  }
}

void RadiatorClimate::save_integral_() {
  if (std::isnan(this->integral_))
    return;
  if (!std::isnan(this->saved_integral_) && std::fabs(this->integral_ - this->saved_integral_) < 0.01f)
    return;
  this->saved_integral_ = this->integral_;
  this->integral_pref_.save(&this->saved_integral_);
}

const RadiatorClimate::Preset *RadiatorClimate::find_preset_(climate::ClimatePreset preset) const {
  for (const auto &entry : this->presets_) {
    if (entry.preset == preset)
      return &entry;
  }
  return nullptr;
}

climate::ClimateTraits RadiatorClimate::traits() {
  auto traits = climate::ClimateTraits();
  uint32_t flags = climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE | climate::CLIMATE_SUPPORTS_ACTION;
  if (this->humidity_sensor_ != nullptr)
    flags |= climate::CLIMATE_SUPPORTS_CURRENT_HUMIDITY;
  traits.add_feature_flags(flags);
  traits.set_supported_modes({climate::CLIMATE_MODE_OFF, climate::CLIMATE_MODE_HEAT});
  if (!this->presets_.empty()) {
    traits.add_supported_preset(climate::CLIMATE_PRESET_NONE);
    for (const auto &entry : this->presets_)
      traits.add_supported_preset(entry.preset);
  }
  traits.set_visual_min_temperature(10.0f);
  traits.set_visual_max_temperature(28.0f);
  traits.set_visual_target_temperature_step(0.5f);
  traits.set_visual_current_temperature_step(0.1f);
  return traits;
}

void RadiatorClimate::dump_config() {
  LOG_CLIMATE("", "Radiator thermostat", this);
  ESP_LOGCONFIG(TAG,
                "  P: %.0f%% per deg, I: %.0f%% per deg-hour, every %.0f s\n"
                "  Deadband: %.1f deg, smallest valve move: %.0f%%, at most %.0f%% open",
                this->kp_ * 100, this->ki_ * 100, this->get_update_interval() / 1000.0f, this->deadband_,
                this->min_step_ * 100, this->max_position_ * 100);
  for (const auto &entry : this->presets_) {
    if (std::isnan(entry.valve_position)) {
      ESP_LOGCONFIG(TAG, "  Preset %s: %.1f deg", LOG_STR_ARG(climate::climate_preset_to_string(entry.preset)),
                    entry.target);
    } else {
      ESP_LOGCONFIG(TAG, "  Preset %s: %.1f deg, valve starts at %.0f%%",
                    LOG_STR_ARG(climate::climate_preset_to_string(entry.preset)), entry.target,
                    entry.valve_position * 100);
    }
  }
  if (!std::isnan(this->integral_)) {
    ESP_LOGCONFIG(TAG, "  Integral: %.0f%%", this->integral_ * 100);
  }
}

}  // namespace esphome::valve_actuator

#endif  // USE_CLIMATE
