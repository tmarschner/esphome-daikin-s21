#pragma once

#include <bitset>
#include <compare>
#include <functional>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <vector>
#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "daikin_s21_fan_modes.h"
#include "daikin_s21_queries.h"
#include "daikin_s21_serial.h"

namespace esphome::daikin_s21 {

enum ProtocolVersion {
  ProtocolUndetected,
  ProtocolUnknown,
  Protocol0,
  Protocol2,
  Protocol3_0,
  Protocol3_1,
  Protocol3_2,
  Protocol3_4,
};

/**
 * Class representing a temperature in degrees C scaled by 10, the most granular internal temperature measurement format
 */
class DaikinC10 {
 public:
  constexpr DaikinC10() = default;

  template <typename T, typename std::enable_if_t<std::is_floating_point_v<T>, bool> = true>
  constexpr DaikinC10(const T valf) : value((static_cast<int16_t>(valf * 10 * 2) + 1) / 2) {} // round to nearest 0.1C

  template <typename T, typename std::enable_if_t<std::is_integral_v<T>, bool> = true>
  constexpr DaikinC10(const T vali) : value(vali) {}

  explicit constexpr operator float() const { return value / 10.0F; }
  explicit constexpr operator int16_t() const { return value; }
  constexpr float f_degc() const { return static_cast<float>(*this); }
  constexpr float f_degf() const { return celsius_to_fahrenheit(static_cast<float>(*this)); }

  constexpr bool operator==(const DaikinC10 &other) const = default;

 private:
  int16_t value{};
};

/**
 * Unit state (RzB2) bitfield decoder
 */
class DaikinUnitState {
 public:
  constexpr DaikinUnitState(const uint8_t value = 0U) : raw(value) {}
  constexpr bool powerful() const { return (this->raw & 0x1) != 0; }
  constexpr bool defrost() const { return (this->raw & 0x2) != 0; }
  constexpr bool active() const { return (this->raw & 0x4) != 0; }
  constexpr bool online() const { return (this->raw & 0x8) != 0; }
  uint8_t raw{};
};

/**
 * System state (RzC3) bitfield decoder
 */
class DaikinSystemState {
 public:
  constexpr DaikinSystemState(const uint8_t value = 0U) : raw(value) {}
  constexpr bool locked() const { return (this->raw & 0x01) != 0; }
  constexpr bool active() const { return (this->raw & 0x04) != 0; }
  constexpr bool defrost() const { return (this->raw & 0x08) != 0; }
  constexpr bool multizone_conflict() const { return (this->raw & 0x20) != 0; }
  uint8_t raw{};
};

struct DaikinSettings {
  climate::ClimateMode mode{climate::CLIMATE_MODE_OFF};
  DaikinC10 setpoint{23};
  climate::ClimateSwingMode swing{climate::CLIMATE_SWING_OFF};
  DaikinFanMode fan{DaikinFanMode::Auto};
  climate::ClimatePreset preset{climate::CLIMATE_PRESET_NONE};

  constexpr bool operator==(const DaikinSettings &other) const = default;
};

class DaikinS21 : public PollingComponent {
 public:
  DaikinS21(DaikinSerial * const serial) : serial(*serial) {} // required in config, non-null

  void setup() override;
  void update() override;
  void dump_config() override;
  void set_debug(bool set) { this->debug = set; }

  // external command action
  void set_climate_settings(const DaikinSettings &settings);

  std::function<void(void)> climate_callback{};
  std::function<void(DaikinUnitState, DaikinSystemState)> binary_sensor_callback{};

  // value accessors
  bool is_ready() { return this->ready.all(); }
  const DaikinSettings& get_climate_settings() { return this->active; };
  climate::ClimateMode get_climate_mode() { return this->active.mode; }
  climate::ClimateAction get_climate_action() { return this->action_resolved; }
  auto get_setpoint() { return this->active.setpoint.f_degc(); }
  auto get_temp_inside() { return this->temp_inside.f_degc(); }
  auto get_temp_outside() { return this->temp_outside.f_degc(); }
  auto get_temp_coil() { return this->temp_coil.f_degc(); }
  auto get_fan_rpm() { return this->fan_rpm; }
  auto get_swing_vertical_angle() { return this->swing_vertical_angle; }
  auto get_compressor_frequency() { return this->compressor_hz; }
  auto get_humidity() { return this->humidity; }
  auto get_demand() { return this->demand; }

  // callbacks for serial events
  void handle_serial_result(DaikinSerial::Result result, std::span<const uint8_t> response = {});
  void handle_serial_idle();

 protected:
  DaikinSerial &serial;

  void dump_state();

  enum ReadyCommand : uint8_t {
    ReadyProtocolVersion,
    ReadySensorReadout,
    ReadyCapabilities,
    ReadyBasic,
    ReadyCount, // just for bitset sizing
  };
  std::bitset<ReadyCount> ready{};

  // communication state
  bool is_free_run() const {
    return this->get_update_interval() == 0;
  }
  void trigger_cycle();
  void start_cycle();
  bool is_query_active(std::string_view query) const {
    return std::ranges::find(this->queries, query) != this->queries.end();
  }
  void prune_query(std::string_view query);
  void refine_queries();
  void parse_ack(std::span<const uint8_t> response);
  bool cycle_triggered{};
  bool cycle_active{};
  std::vector<std::string_view> queries{};
  std::vector<std::string_view>::iterator current_query{};
  std::string_view tx_command{};  // used when matching responses - backing value must have persistent lifetime across serial state machine runs

  // debugging support
  bool debug{};
  std::unordered_map<std::string, std::vector<uint8_t>> val_cache{};
  std::vector<std::string_view> nak_queries{};
  uint32_t last_state_dump_ms{};
  uint32_t cycle_time_start_ms{};
  uint32_t cycle_time_ms{};

  // settings
  DaikinSettings active{};
  DaikinSettings pending{ .mode = climate::CLIMATE_MODE_AUTO }; // unsupported sentinel value, see set_climate_settings
  bool activate_climate{};
  bool activate_swing_mode{};
  bool activate_preset{};

  // current values
  climate::ClimateAction action_reported = climate::CLIMATE_ACTION_OFF; // raw readout
  climate::ClimateAction action_resolved = climate::CLIMATE_ACTION_OFF; // corrected at end of cycle
  DaikinC10 temp_inside{};
  DaikinC10 temp_target{};
  DaikinC10 temp_outside{};
  DaikinC10 temp_coil{};
  uint16_t fan_rpm{};
  int16_t swing_vertical_angle{};
  uint8_t compressor_hz{};
  uint8_t humidity{50};
  uint8_t demand{};
  DaikinUnitState unit_state{};
  DaikinSystemState system_state{};
  enum Modifier : uint8_t {
    ModifierQuiet,    // outdoor unit fan/compressor limit
    ModifierEcono,    // limits demand for power consumption
    ModifierPowerful, // maximum output (20 minute timeout), mutaully exclusive with quiet and econo
    ModifierComfort,  // fan angle depends on heating/cooling action
    ModifierStreamer, // electron emitter decontamination?
    ModifierSensor,   // "intelligent eye" PIR occupancy setpoint offset
    ModifierLED,      // the sensor LED is on
    ModifierCount,    // just for bitset sizing
  };
  std::bitset<ModifierCount> modifiers{};

  // protocol support
  bool determine_protocol_version();
  struct DetectResponses {
    std::array<uint8_t,4> G8{};
    std::array<uint8_t,4> GC{};
    uint16_t GY00{};
    std::array<uint8_t,4> M{};
    std::array<uint8_t,4> V{};
    char G2_model_info{};
  } detect_responses;
  ProtocolVersion protocol_version{ProtocolUndetected};
  bool support_swing{};
  bool support_horizontal_swing{};
  bool support_humidity{};

  // helpers
  climate::ClimateAction resolve_climate_action();
};

} // namespace esphome::daikin_s21
