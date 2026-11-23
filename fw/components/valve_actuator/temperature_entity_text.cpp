#include "temperature_entity_text.h"

#if defined(USE_CLIMATE) && defined(USE_TEXT)

#include <cstring>

#include "esphome/core/log.h"

namespace esphome::valve_actuator {

static const char *const TAG = "valve_actuator.text";

static constexpr uint32_t PREF_VERSION = 0x54454E31;  // "TEN1"

// "" or a sensor entity ID: "sensor." and then lowercase letters, digits, '_'.
static bool valid_entity_id(const std::string &value) {
  if (value.empty())
    return true;
  if (value.size() <= 7 || value.compare(0, 7, "sensor.") != 0)
    return false;
  for (size_t i = 7; i < value.size(); i++) {
    char c = value[i];
    if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_'))
      return false;
  }
  return true;
}

void TemperatureEntityText::setup() {
  this->pref_ = this->make_entity_preference<Stored>(PREF_VERSION);
  std::string value = this->initial_value_;
  Stored stored{};
  if (this->pref_.load(&stored)) {
    stored.value[MAX_LENGTH] = '\0';
    value = stored.value;
  }
  this->apply_(value);
}

void TemperatureEntityText::control(const std::string &value) {
  std::string entity_id = value;
  // Forgive stray spaces from copy and paste.
  while (!entity_id.empty() && entity_id.back() == ' ')
    entity_id.pop_back();
  while (!entity_id.empty() && entity_id.front() == ' ')
    entity_id.erase(0, 1);
  if (!valid_entity_id(entity_id) || entity_id.size() > MAX_LENGTH) {
    ESP_LOGW(TAG, "'%s' isn't a sensor entity ID like sensor.room_temperature; keeping '%s'", value.c_str(),
             this->state.c_str());
    this->publish_state(this->state);
    return;
  }
  Stored stored{};
  std::strncpy(stored.value, entity_id.c_str(), MAX_LENGTH);
  this->pref_.save(&stored);
  this->apply_(entity_id);
}

void TemperatureEntityText::apply_(const std::string &value) {
  this->publish_state(value);
  if (this->thermostat_ == nullptr)
    return;
  if (this->preset_.has_value()) {
    this->thermostat_->set_preset_temperature_entity(*this->preset_, value);
  } else {
    this->thermostat_->set_temperature_entity(value);
  }
}

void TemperatureEntityText::dump_config() { LOG_TEXT("", "Temperature sensor entity", this); }

}  // namespace esphome::valve_actuator

#endif  // USE_CLIMATE && USE_TEXT
