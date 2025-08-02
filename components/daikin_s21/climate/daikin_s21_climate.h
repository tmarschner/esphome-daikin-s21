#pragma once

#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"
#include "../daikin_s21_fan_modes.h"
#include "../s21.h"

namespace esphome {
namespace daikin_s21 {

class DaikinS21Climate : public climate::Climate,
                         public Component,
                         public Parented<DaikinS21> {
 public:
  void setup() override;
  void dump_config() override;
  void control(const climate::ClimateCall &call) override;
  void command_timeout_handler();
  void update_handler();

  void set_room_sensor(sensor::Sensor *sensor) { this->room_sensor_ = sensor; }
  void set_setpoint_interval(uint16_t seconds) { this->setpoint_interval_s = seconds; };
  void set_supported_modes_override(std::set<climate::ClimateMode> modes) { this->supported_modes_override_ = std::move(modes); }
  void set_supported_presets_override(std::set<climate::ClimatePreset> presets) { this->supported_presets_override_ = std::move(presets); }
  void set_supports_current_humidity(bool supports_current_humidity) { this->supports_current_humidity_ = supports_current_humidity; }

 protected:
  static constexpr const char * command_timeout_name = "cmd";
  static constexpr uint32_t state_publication_timeout_ms{8 * 1000}; // experimentally determined with fudge factor

  climate::ClimateTraits traits() override;
  optional<std::set<climate::ClimateMode>> supported_modes_override_{};
  optional<std::set<climate::ClimatePreset>> supported_presets_override_{};
  bool supports_current_humidity_{false};

  sensor::Sensor *room_sensor_{};
  uint16_t setpoint_interval_s{};
  uint32_t last_setpoint_check_ms{millis()};

  bool command_active{};  // ESPHome could use a is_timeout_active()...
  DaikinSettings commanded{};

  ESPPreferenceObject auto_setpoint_pref;
  ESPPreferenceObject cool_setpoint_pref;
  ESPPreferenceObject heat_setpoint_pref;

  void set_custom_fan_mode(DaikinFanMode mode);
  bool use_room_sensor();
  bool room_sensor_unit_is_valid();
  float room_sensor_degc();
  float get_effective_current_temperature();
  float get_room_temp_offset();
  float calc_s21_setpoint();
  float s21_setpoint_variance();
  void save_setpoint(float value, ESPPreferenceObject &pref);
  void save_setpoint(float value);
  optional<float> load_setpoint(ESPPreferenceObject &pref);
  optional<float> load_setpoint();
  bool should_check_setpoint();
  void set_s21_climate();
  void set_command_timeout(uint32_t delay_ms = state_publication_timeout_ms);
};

}  // namespace daikin_s21
}  // namespace esphome
