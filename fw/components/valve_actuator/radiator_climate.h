#pragma once

#include "esphome/core/defines.h"
#ifdef USE_CLIMATE

#include <string>
#include <vector>

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"

#include "valve_actuator.h"

namespace esphome::valve_actuator {

// A thermostat for the radiator: a PI controller that sets the valve position
// from the room temperature.
//
// The temperature comes from a Home Assistant sensor chosen at runtime (by
// entity ID, from a text entity), optionally a different one per preset, or
// else from a sensor on the device.
//
// Built to move the valve as little as possible: the controller runs once per
// update_interval, the integral only changes outside a deadband around the
// target, and the valve only moves when the result differs from where it is
// by at least min_step.
//
// Moving the valve directly (Home Assistant, the board buttons, an
// automation) holds it there: the controller steps back until the next change
// to the thermostat's mode, target or preset.
class RadiatorClimate : public climate::Climate, public PollingComponent {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_valve(ValveActuator *valve) { this->valve_ = valve; }
  // Used when no Home Assistant sensor is chosen.
  void set_sensor(sensor::Sensor *sensor) { this->sensor_ = sensor; }
  // Home Assistant sensor to follow (entity ID, "" for none), for all presets
  // or for one. Works at runtime: a new entity is subscribed to on the spot.
  void set_temperature_entity(const std::string &entity_id);
  void set_preset_temperature_entity(climate::ClimatePreset preset, const std::string &entity_id);
  void set_humidity_sensor(sensor::Sensor *sensor) { this->humidity_sensor_ = sensor; }
  void set_held_sensor(binary_sensor::BinarySensor *sensor) { this->held_sensor_ = sensor; }
  void set_default_target_temperature(float celsius) { this->default_target_ = celsius; }
  // valve_position: where the valve usually sits at this preset's target
  // (NAN if unknown). Used as the starting point when the preset is chosen.
  void add_preset(climate::ClimatePreset preset, float target, float valve_position) {
    this->presets_.push_back({preset, target, valve_position, {}});
  }
  // Gains also work at runtime (from template numbers).
  void set_kp(float per_degree) { this->kp_ = per_degree; }
  void set_ki(float per_degree_hour) { this->ki_ = per_degree_hour; }
  void set_deadband(float degrees) { this->deadband_ = degrees; }
  void set_min_step(float fraction) { this->min_step_ = fraction; }
  void set_max_position(float fraction) { this->max_position_ = fraction; }

 protected:
  struct Preset {
    climate::ClimatePreset preset;
    float target;
    float valve_position;
    std::string entity_id;  // "" = the main one
  };
  struct Subscription {
    std::string entity_id;
    float value;
  };

  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;

  const Preset *find_preset_(climate::ClimatePreset preset) const;
  const std::string &active_entity_() const;
  void subscribe_(const std::string &entity_id);
  void on_entity_state_(size_t index, float value);
  bool refresh_temperature_();
  void source_changed_();
  void set_held_(bool held);
  void move_valve_(float output, bool force);
  void update_action_();
  void save_integral_();

  ValveActuator *valve_{nullptr};
  sensor::Sensor *sensor_{nullptr};
  sensor::Sensor *humidity_sensor_{nullptr};
  binary_sensor::BinarySensor *held_sensor_{nullptr};
  std::vector<Preset> presets_;
  std::string entity_id_;
  std::vector<Subscription> subscriptions_;
  bool setup_done_{false};

  float default_target_{21.0f};
  float kp_{0.2f};   // valve fraction per degree below target
  float ki_{0.05f};  // valve fraction per degree-hour
  float deadband_{0.2f};
  float min_step_{0.05f};
  float max_position_{1.0f};

  float integral_{NAN};  // valve fraction; NAN until the first update
  float saved_integral_{NAN};
  ESPPreferenceObject integral_pref_;
  uint32_t last_update_ms_{0};
  bool held_{false};
  bool restoring_{false};
  bool warned_no_temperature_{false};
};

}  // namespace esphome::valve_actuator

#endif  // USE_CLIMATE
