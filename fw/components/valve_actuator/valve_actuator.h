#pragma once

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/valve/valve.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"

#include "tmc2209.h"

namespace esphome::valve_actuator {

// A radiator valve turned by a stepper on the T-Motor's TMC2209.
// Motion is STEP/DIR generated in loop(); the driver is configured over UART.
// Position 0 is the closed end stop and 1 the open end, `travel` apart.
//
// Calibration finds the end stops without switches: it drives into each one in
// StealthChop and watches StallGuard (SG_RESULT, the driver's load reading,
// which drops as the load rises). A stall is SG_RESULT staying below
// `stall_threshold` times the value measured while running free.
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
  void set_travel(float degrees) {
    this->travel_deg_ = degrees;
    this->travel_configured_ = true;
  }
  void set_max_speed(float deg_per_s) { this->max_speed_deg_s_ = deg_per_s; }
  void set_acceleration(float deg_per_s2) { this->accel_deg_s2_ = deg_per_s2; }
  void set_hold_when_idle(bool hold) { this->hold_when_idle_ = hold; }
  void set_open_limit(binary_sensor::BinarySensor *sensor) { this->open_limit_ = sensor; }
  void set_closed_limit(binary_sensor::BinarySensor *sensor) { this->closed_limit_ = sensor; }

  // Calibration. The speed and threshold setters also work at runtime (from a
  // template number); they apply from the next calibration.
  void set_homing_enabled(bool enabled) { this->homing_enabled_ = enabled; }
  void set_home_on_boot(bool on_boot) { this->home_on_boot_ = on_boot; }
  void set_homing_speed(float deg_per_s) { this->homing_speed_deg_s_ = deg_per_s; }
  void set_homing_current(float amps) { this->homing_current_ = amps; }
  void set_stall_threshold(float fraction) { this->stall_threshold_ = fraction; }
  void set_max_travel(float degrees) { this->max_travel_deg_ = degrees; }
  void set_backoff(float degrees) { this->backoff_deg_ = degrees; }

  void set_stallguard_sensor(sensor::Sensor *sensor) { this->stallguard_sensor_ = sensor; }
  void set_travel_sensor(sensor::Sensor *sensor) { this->travel_sensor_ = sensor; }
  void set_status_sensor(text_sensor::TextSensor *sensor) { this->status_sensor_ = sensor; }

  // Find the end stops again, then return to the current target position.
  void calibrate();
  bool is_calibrated() const { return this->calibrated_; }

 protected:
  enum class Homing : uint8_t {
    IDLE,
    SETTLE,          // motor powered at standstill so StealthChop can tune itself
    RUNUP,           // move away from the closed end so the seek can measure free running
    SEEK_CLOSED,
    BACKOFF,         // move off the closed end stop
    SEEK_OPEN,       // only when `travel` isn't configured: measure it
  };

  void control(const valve::ValveCall &call) override;

  bool connect_driver_();
  void poll_driver_();
  void driver_lost_();
  void log_driver_status_();

  void move_to_(int32_t target);
  void start_move_(int32_t target, float speed_limit);
  void stop_();
  void run_motion_();
  void end_move_();
  void go_idle_();
  bool limit_tripped_(int8_t direction) const;
  void publish_position_(bool save);

  void start_calibration_();
  void calibration_step_();
  void finish_calibration_();
  void abort_calibration_();
  void restore_motion_settings_();
  bool seeking_() const { return this->homing_ == Homing::SEEK_CLOSED || this->homing_ == Homing::SEEK_OPEN; }
  void start_seek_(int8_t direction);

  void poll_stallguard_();
  void on_stallguard_(uint16_t value);

  void set_status_(const char *status);
  void set_travel_steps_(int32_t steps);

  GPIOPin *step_pin_{nullptr};
  GPIOPin *dir_pin_{nullptr};
  GPIOPin *enable_pin_{nullptr};
  GPIOPin *clk_pin_{nullptr};
  GPIOPin *spread_pin_{nullptr};
  binary_sensor::BinarySensor *open_limit_{nullptr};
  binary_sensor::BinarySensor *closed_limit_{nullptr};
  sensor::Sensor *stallguard_sensor_{nullptr};
  sensor::Sensor *travel_sensor_{nullptr};
  text_sensor::TextSensor *status_sensor_{nullptr};

  TMC2209 driver_;
  TMC2209::Settings settings_{0.12f, 1.0f, 0.3f, 16, true, false};
  optional<uint8_t> fixed_address_{};
  bool driver_ok_{false};
  uint32_t last_driver_poll_{0};

  uint16_t steps_per_revolution_{200};
  float gear_ratio_{1.0f};
  float travel_deg_{360.0f};
  bool travel_configured_{false};
  float max_speed_deg_s_{90.0f};
  float accel_deg_s2_{180.0f};
  bool hold_when_idle_{false};

  bool homing_enabled_{false};
  bool home_on_boot_{true};
  float homing_speed_deg_s_{45.0f};
  float homing_current_{0.0f};  // 0 = run current
  float stall_threshold_{0.5f};
  float max_travel_deg_{1080.0f};
  float backoff_deg_{5.0f};

  // Derived in setup(), in microsteps.
  float steps_per_deg_{0};
  int32_t travel_steps_{0};
  float max_speed_{0};  // steps/s
  float accel_{0};      // steps/s^2
  float min_speed_{0};  // steps/s
  ESPPreferenceObject travel_pref_;

  // Motion.
  int32_t position_{0};
  int32_t target_{0};
  int8_t direction_{0};  // of the last step, for DIR changes
  float speed_{0};       // steps/s
  float speed_limit_{0};  // steps/s, for the current move
  bool moving_{false};
  uint32_t last_update_us_{0};
  uint32_t last_step_us_{0};
  uint32_t last_publish_ms_{0};
  HighFrequencyLoopRequester high_freq_;

  // Calibration.
  Homing homing_{Homing::IDLE};
  bool calibration_requested_{false};
  bool calibrated_{false};
  bool end_found_{false};
  const char *calibration_result_{"Not calibrated"};
  const char *status_{nullptr};
  uint32_t settle_start_ms_{0};
  float homing_speed_{0};  // steps/s
  optional<float> resume_position_{};  // where to go once calibrated

  // StallGuard polling.
  bool sg_active_{false};
  uint32_t sg_interval_us_{0};
  uint32_t last_sg_request_us_{0};
  uint32_t last_sg_publish_ms_{0};
  uint8_t sg_failures_{0};
  uint8_t sg_samples_{0};
  uint8_t sg_low_count_{0};
  uint32_t sg_sum_{0};
  float sg_baseline_{0};
  uint16_t sg_min_{0};
};

}  // namespace esphome::valve_actuator
