#pragma once

#include "esphome/core/defines.h"
#if defined(USE_CLIMATE) && defined(USE_TEXT)

#include <string>

#include "esphome/components/text/text.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"

#include "radiator_climate.h"

namespace esphome::valve_actuator {

// A text entity holding the entity ID of the Home Assistant sensor the
// thermostat takes its temperature from, for all presets or for one. Empty
// means none (for a preset: use the main one). Saved across reboots.
class TemperatureEntityText : public text::Text, public Component {
 public:
  static constexpr size_t MAX_LENGTH = 100;

  void setup() override;
  void dump_config() override;
  // Before the thermostat, so it subscribes to the restored entity at setup.
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  void set_thermostat(RadiatorClimate *thermostat) { this->thermostat_ = thermostat; }
  void set_thermostat(RadiatorClimate *thermostat, climate::ClimatePreset preset) {
    this->thermostat_ = thermostat;
    this->preset_ = preset;
  }
  void set_initial_value(const char *value) { this->initial_value_ = value; }

 protected:
  struct Stored {
    char value[MAX_LENGTH + 1];
  };

  void control(const std::string &value) override;
  void apply_(const std::string &value);

  RadiatorClimate *thermostat_{nullptr};
  optional<climate::ClimatePreset> preset_{};  // none = for all presets
  const char *initial_value_{""};
  ESPPreferenceObject pref_;
};

}  // namespace esphome::valve_actuator

#endif  // USE_CLIMATE && USE_TEXT
