#pragma once

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/valve/valve.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"

#include "tmc2209.h"

namespace esphome::valve_actuator {

// A radiator valve turned by a stepper on the T-Motor's TMC2209.
// Motion is STEP/DIR generated in loop(); the driver is configured over UART.
// Position 0 is closed, 1 is open, mapped linearly onto `travel`.
class ValveActuator : public valve::Valve, public Component, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  valve::ValveTraits get_traits() override;

  void set_step_pin(GPIOPin *pin) { this->step_pin_ = pin; }
  void set_dir_pin(GPIOPin *pin) { this->dir_pin_ = pin; }
  void set_enable_pin(GPIOPin *pin) { this->enable_pin_ = pin; }
  void set_clk_pin(GPIOPin *pin) { this->clk_pin_ = pin; }
  void set_spread_pin(GPIOPin *pin) { this->spread_pin_ = pin; }
  void set_address(uint8_t address) { this->fixed_address_ = address; }
  void set_sense_resistor(float ohm) { this->settings_.sense_resistor = ohm; }
  void set_run_current(float amps) { this->settings_.run_current = amps; }
  void set_hold_current(float fraction) { this->settings_.hold_fraction = fraction; }
  void set_microsteps(uint16_t microsteps) { this->settings_.microsteps = microsteps; }
  void set_spread_cycle(bool spread_cycle) { this->settings_.spread_cycle = spread_cycle; }
  void set_invert_direction(bool invert) { this->settings_.invert_direction = invert; }
  void set_steps_per_revolution(uint16_t steps) { this->steps_per_revolution_ = steps; }
  void set_gear_ratio(float ratio) { this->gear_ratio_ = ratio; }
  void set_travel(float degrees) { this->travel_deg_ = degrees; }
  void set_max_speed(float deg_per_s) { this->max_speed_deg_s_ = deg_per_s; }
  void set_acceleration(float deg_per_s2) { this->accel_deg_s2_ = deg_per_s2; }
  void set_hold_when_idle(bool hold) { this->hold_when_idle_ = hold; }
  void set_open_limit(binary_sensor::BinarySensor *sensor) { this->open_limit_ = sensor; }
  void set_closed_limit(binary_sensor::BinarySensor *sensor) { this->closed_limit_ = sensor; }

 protected:
  void control(const valve::ValveCall &call) override;

  bool connect_driver_();
  void poll_driver_();
  void log_driver_status_();

  void move_to_(int32_t target);
  void stop_();
  void run_motion_();
  void finish_move_();
  bool limit_tripped_(int8_t direction) const;
  void publish_position_(bool save);

  GPIOPin *step_pin_{nullptr};
  GPIOPin *dir_pin_{nullptr};
  GPIOPin *enable_pin_{nullptr};
  GPIOPin *clk_pin_{nullptr};
  GPIOPin *spread_pin_{nullptr};
  binary_sensor::BinarySensor *open_limit_{nullptr};
  binary_sensor::BinarySensor *closed_limit_{nullptr};

  TMC2209 driver_;
  TMC2209::Settings settings_{0.12f, 1.0f, 0.3f, 16, true, false};
  optional<uint8_t> fixed_address_{};
  bool driver_ok_{false};
  uint32_t last_driver_poll_{0};

  uint16_t steps_per_revolution_{200};
  float gear_ratio_{1.0f};
  float travel_deg_{360.0f};
  float max_speed_deg_s_{90.0f};
  float accel_deg_s2_{180.0f};
  bool hold_when_idle_{false};

  // Derived in setup(), in microsteps.
  float steps_per_deg_{0};
  int32_t travel_steps_{0};
  float max_speed_{0};  // steps/s
  float accel_{0};      // steps/s^2
  float min_speed_{0};  // steps/s

  int32_t position_{0};
  int32_t target_{0};
  int8_t direction_{0};  // of the last step, for DIR changes
  float speed_{0};       // steps/s
  bool moving_{false};
  uint32_t last_update_us_{0};
  uint32_t last_step_us_{0};
  uint32_t last_publish_ms_{0};
  HighFrequencyLoopRequester high_freq_;
};

}  // namespace esphome::valve_actuator
