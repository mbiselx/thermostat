#include "radiator_climate.h"

#ifdef USE_CLIMATE

#include <algorithm>
#include <cmath>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#ifdef USE_API
#include "esphome/components/api/api_server.h"
#ifdef USE_API_HOMEASSISTANT_STATES
#include "esphome/components/api/api_connection.h"
#include "esphome/components/api/api_pb2.h"
#endif
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

  if (this->sensor_ != nullptr) {
    this->sensor_->add_on_state_callback([this](float) {
      if (this->active_entity_().empty())
        this->source_changed_();
    });
  }
  // Entities chosen before setup (text entities restore first) get subscribed
  // here, before Home Assistant connects.
  this->subscribe_(this->entity_id_);
  for (const auto &entry : this->presets_)
    this->subscribe_(entry.entity_id);
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
  this->refresh_temperature_();
  if (this->held_sensor_ != nullptr)
    this->held_sensor_->publish_state(false);
  this->setup_done_ = true;
}

// --- Temperature source --------------------------------------------------------

void RadiatorClimate::set_temperature_entity(const std::string &entity_id) {
  if (entity_id == this->entity_id_)
    return;
  this->entity_id_ = entity_id;
  if (this->setup_done_) {
    ESP_LOGI(TAG, "Temperature sensor: %s", entity_id.empty() ? "none" : entity_id.c_str());
    this->subscribe_(entity_id);
    this->source_changed_();
  }
}

void RadiatorClimate::set_preset_temperature_entity(climate::ClimatePreset preset, const std::string &entity_id) {
  for (auto &entry : this->presets_) {
    if (entry.preset != preset)
      continue;
    if (entry.entity_id == entity_id)
      return;
    entry.entity_id = entity_id;
    if (this->setup_done_) {
      ESP_LOGI(TAG, "Temperature sensor for %s: %s", LOG_STR_ARG(climate::climate_preset_to_string(preset)),
               entity_id.empty() ? "the main one" : entity_id.c_str());
      this->subscribe_(entity_id);
      this->source_changed_();
    }
    return;
  }
  ESP_LOGW(TAG, "No preset %s to give a temperature sensor", LOG_STR_ARG(climate::climate_preset_to_string(preset)));
}

// The current preset's sensor if it has one, else the main one ("" = none).
const std::string &RadiatorClimate::active_entity_() const {
  if (this->preset.has_value()) {
    const Preset *entry = this->find_preset_(*this->preset);
    if (entry != nullptr && !entry->entity_id.empty())
      return entry->entity_id;
  }
  return this->entity_id_;
}

// Subscriptions can't be dropped, so every entity ever chosen stays subscribed
// until the next reboot; only the active one is used.
void RadiatorClimate::subscribe_(const std::string &entity_id) {
  if (entity_id.empty())
    return;
  for (const auto &sub : this->subscriptions_) {
    if (sub.entity_id == entity_id)
      return;
  }
  size_t index = this->subscriptions_.size();
  this->subscriptions_.push_back({entity_id, NAN});
#ifdef USE_API_HOMEASSISTANT_STATES
  api::global_api_server->subscribe_home_assistant_state(
      entity_id, {}, [this, index](StringRef state) {
        auto value = parse_number<float>(state.c_str());
        if (!value.has_value()) {
          ESP_LOGW(TAG, "'%s': Can't use '%s' as a temperature", this->subscriptions_[index].entity_id.c_str(),
                   state.c_str());
        }
        this->on_entity_state_(index, value.value_or(NAN));
      });
  if (this->setup_done_) {
    // Home Assistant asked for the subscriptions when it connected; tell it
    // about this one now instead of at the next connection.
    api::SubscribeHomeAssistantStateResponse request;
    request.entity_id = StringRef(this->subscriptions_[index].entity_id);
    request.attribute = StringRef("");
    request.once = false;
    for (const auto &client : api::global_api_server->active_clients()) {
      if (!client->send_message(request)) {
        ESP_LOGW(TAG, "Couldn't subscribe to %s yet; it will be at the next connection", entity_id.c_str());
      }
    }
  }
#else
  ESP_LOGW(TAG, "Can't follow %s: no Home Assistant API", entity_id.c_str());
#endif
}

void RadiatorClimate::on_entity_state_(size_t index, float value) {
  this->subscriptions_[index].value = value;
  if (this->subscriptions_[index].entity_id == this->active_entity_())
    this->source_changed_();
}

// Takes the temperature from the active source. Returns whether that's the
// first valid reading.
bool RadiatorClimate::refresh_temperature_() {
  const std::string &entity = this->active_entity_();
  float temperature = NAN;
  if (!entity.empty()) {
    for (const auto &sub : this->subscriptions_) {
      if (sub.entity_id == entity)
        temperature = sub.value;
    }
  } else if (this->sensor_ != nullptr) {
    temperature = this->sensor_->state;
  }
  bool first = std::isnan(this->current_temperature) && !std::isnan(temperature);
  this->current_temperature = temperature;
  return first;
}

void RadiatorClimate::source_changed_() {
  bool first = this->refresh_temperature_();
  if (first && this->setup_done_) {
    this->update();  // react to the first reading instead of waiting
  } else {
    this->publish_state();
  }
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
  // The preset may come with its own sensor.
  this->refresh_temperature_();
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
  ESP_LOGCONFIG(TAG, "  Temperature: %s", !this->entity_id_.empty() ? this->entity_id_.c_str()
                                          : this->sensor_ != nullptr  ? "device sensor"
                                                                      : "none");
  for (const auto &entry : this->presets_) {
    ESP_LOGCONFIG(TAG, "  Preset %s: %.1f deg", LOG_STR_ARG(climate::climate_preset_to_string(entry.preset)),
                  entry.target);
    if (!std::isnan(entry.valve_position)) {
      ESP_LOGCONFIG(TAG, "    Valve starts at %.0f%%", entry.valve_position * 100);
    }
    if (!entry.entity_id.empty()) {
      ESP_LOGCONFIG(TAG, "    Temperature: %s", entry.entity_id.c_str());
    }
  }
  if (!std::isnan(this->integral_)) {
    ESP_LOGCONFIG(TAG, "  Integral: %.0f%%", this->integral_ * 100);
  }
}

}  // namespace esphome::valve_actuator

#endif  // USE_CLIMATE
