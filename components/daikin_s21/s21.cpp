#include <cinttypes>
#include <numeric>
#include <ranges>
#include "s21.h"
#include "utils.h"

using namespace esphome;

namespace esphome::daikin_s21 {

static const char *const TAG = "daikin_s21";

uint8_t climate_mode_to_daikin(const climate::ClimateMode mode) {
  switch (mode) {
      break;
    case climate::CLIMATE_MODE_HEAT:
      return '4';
    case climate::CLIMATE_MODE_COOL:
      return '3';
    case climate::CLIMATE_MODE_FAN_ONLY:
      return '6';
    case climate::CLIMATE_MODE_DRY:
      return '2';
    case climate::CLIMATE_MODE_OFF:
    case climate::CLIMATE_MODE_HEAT_COOL:
    case climate::CLIMATE_MODE_AUTO:
    default:
      return '1';
  }
}

climate::ClimateMode daikin_to_climate_mode(const uint8_t mode) {
  switch (mode) {
    case '2': // dry
      return climate::CLIMATE_MODE_DRY;
    case '3': // cool
      return climate::CLIMATE_MODE_COOL;
    case '4': // heat
      return climate::CLIMATE_MODE_HEAT;
    case '6': // fan_only
      return climate::CLIMATE_MODE_FAN_ONLY;
    case '0': // heat_cool cooling
    case '1': // heat_cool setting
    case '7': // heat_cool heating
    default:
      return climate::CLIMATE_MODE_HEAT_COOL;
  }
}

climate::ClimateAction daikin_to_climate_action(const uint8_t action) {
  switch (action) {
    case '2': // dry
      return climate::CLIMATE_ACTION_DRYING;
    case '0': // heat_cool cooling
    case '3': // cool
      return climate::CLIMATE_ACTION_COOLING;
    case '4': // heat
    case '7': // heat_cool heating
      return climate::CLIMATE_ACTION_HEATING;
    case '6': // fan_only
      return climate::CLIMATE_ACTION_FAN;
    case '1': // heat_cool
    default:
      return climate::CLIMATE_ACTION_IDLE;
  }
}

climate::ClimateSwingMode daikin_to_climate_swing_mode(const uint8_t mode) {
  switch (mode) {
    case '1':
      return climate::CLIMATE_SWING_VERTICAL;
    case '2':
      return climate::CLIMATE_SWING_HORIZONTAL;
    case '7':
      return climate::CLIMATE_SWING_BOTH;
    default:
      return climate::CLIMATE_SWING_OFF;
  }
}

uint8_t climate_swing_mode_to_daikin(const climate::ClimateSwingMode mode) {
  switch (mode) {
    case climate::CLIMATE_SWING_BOTH:
      return '7';
    case climate::CLIMATE_SWING_VERTICAL:
      return '1';
    case climate::CLIMATE_SWING_HORIZONTAL:
      return '2';
    case climate::CLIMATE_SWING_OFF:
    default:
      return '0';
  }
}

std::string_view protocol_to_string(const ProtocolVersion version) {
  switch (version) {
    case ProtocolUnknown:
      return "Unknown";
    case Protocol0:
      return "0";
    case Protocol2:
      return "2";
    case Protocol3_0:
      return "3.0";
    case Protocol3_1:
      return "3.1";
    case Protocol3_2:
      return "3.2";
    case Protocol3_4:
      return "3.4";
    case ProtocolUndetected:
    default:
      return "Undetected";
  }
}

int16_t bytes_to_num(std::span<const uint8_t> bytes) {
  // <ones><tens><hundreds><neg/pos>
  int16_t val = 0;
  val = bytes[0] - '0';
  val += (bytes[1] - '0') * 10;
  val += (bytes[2] - '0') * 100;
  if ((bytes.size() > 3) && (bytes[3] == '-')) {
    val *= -1;
  }
  return val;
}

void DaikinS21::setup() {
  // populate initial messages to poll
  // clang-format off
  this->queries = {
      // Protocol version detect
      {StateQuery::OldProtocol, &DaikinS21::handle_nop, true},
      {StateQuery::NewProtocol, &DaikinS21::handle_nop, true},
      {StateQuery::ModelCode, &DaikinS21::handle_nop, true},
      {MiscQuery::Model, &DaikinS21::handle_nop, true}, // some sort of model? always "3E53" for me, regardless of head unit
      {MiscQuery::Version, &DaikinS21::handle_nop, true}, // purportedly another version, always "00C0" for me
      // Basic sensor support
      {StateQuery::InsideOutsideTemperatures, &DaikinS21::handle_state_inside_outside_temperature},
      {EnvironmentQuery::InsideTemperature, &DaikinS21::handle_env_inside_temperature},
      {EnvironmentQuery::LiquidTemperature, &DaikinS21::handle_env_liquid_temperature},
      //{EnvironmentQuery::FanSpeedSetpoint, &DaikinS21::handle_env_fan_speed_setpoint},  // not supported yet, can translate DaikinFanMode to RPM
      {EnvironmentQuery::FanSpeed, &DaikinS21::handle_env_fan_speed},
      {EnvironmentQuery::OutsideTemperature, &DaikinS21::handle_env_outside_temperature},
      //{EnvironmentQuery::LouvreAngleSetpoint, &DaikinS21::handle_env_vertical_swing_angle_setpoint},  // not supported yet
      // Standard
      {StateQuery::Basic, &DaikinS21::handle_state_basic},
      {StateQuery::OptionalFeatures, &DaikinS21::handle_state_optional_features, true},
      {StateQuery::SwingOrHumidity, &DaikinS21::handle_state_swing_or_humidity},
      {StateQuery::SpecialModes, &DaikinS21::handle_state_special_modes},
      {StateQuery::DemandAndEcono, &DaikinS21::handle_state_demand_and_econo},
      {EnvironmentQuery::FanMode, &DaikinS21::handle_env_fan_mode},
      {EnvironmentQuery::TargetTemperature, &DaikinS21::handle_env_target_temperature},
      {EnvironmentQuery::IndoorFrequencyCommandSignal, &DaikinS21::handle_env_indoor_frequency_command_signal},
      {EnvironmentQuery::CompressorFrequency, &DaikinS21::handle_env_compressor_frequency},
      // State
      {EnvironmentQuery::UnitState, &DaikinS21::handle_env_unit_state},
      {EnvironmentQuery::SystemState, &DaikinS21::handle_env_system_state},
      // Redundant
      // {EnvironmentQuery::PowerOnOff, &DaikinS21::handle_env_power_on_off},
      // {EnvironmentQuery::IndoorUnitMode, &DaikinS21::handle_env_indoor_unit_mode},
      // {EnvironmentQuery::TemperatureSetpoint, &DaikinS21::handle_env_temperature_setpoint},
      // {EnvironmentQuery::SwingMode, &DaikinS21::handle_env_swing_mode},
      // {EnvironmentQuery::FanMode, &DaikinS21::handle_env_fan_mode},
      // Unused
      // {StateQuery::OnOffTimer},  // use home assistant for scheduling
      // Not handled yet
      // {StateQuery::ErrorStatus},
  };
  // clang-format on
  this->failed_queries = {};
  this->static_queries = {};
  this->comms_detected = false;
  this->start_poller();
  this->disable_loop();
}

/**
 * Component update loop.
 *
 * Used for deferred work. Printing too much in the timer callback context causes warnings about blocking for too long.
 */
void DaikinS21::loop() {
  this->dump_state(); // use Component::defer if more work items are added
  this->disable_loop();
}

/**
 * PollingComponent update loop.
 *
 * Disable the polling loop (this function) if configured to free run.
 * It executes once on startup
 *
 * Trigger a cycle.
 */
void DaikinS21::update() {
  if (this->is_free_run()) {
    this->stop_poller();
  }
  this->trigger_cycle();
}

void DaikinS21::dump_config() {
  ESP_LOGCONFIG(TAG, "DaikinS21:");
  ESP_LOGCONFIG(TAG, "  Update interval: %" PRIu32, this->get_update_interval());
}

void DaikinS21::set_climate_settings(const DaikinClimateSettings &settings) {
  if ((this->pending.climate.mode != settings.mode) ||
      (this->pending.climate.setpoint != settings.setpoint) ||
      (this->pending.climate.fan != settings.fan)) {

    // If this is the first time settings are being applied, we need to force
    // all of them to be applied to the unit so our state will be in sync with
    // the unit and thus rely on this change detection to work in the future.
    // The pending settings mode has been initialized in the object constructor
    // to the unsupported (by this project) CLIMATE_MODE_AUTO in order to detect
    // this one time condition here.
    if (this->pending.climate.mode == climate::CLIMATE_MODE_AUTO) {
      this->pending.activate_swing_mode = true;
      this->pending.activate_preset = true;
    }

    this->pending.activate_climate = true;
    this->trigger_cycle();
  }

  if (this->pending.climate.swing != settings.swing) {
    this->pending.activate_swing_mode = true;
    this->trigger_cycle();
  }

  if (this->pending.climate.preset != settings.preset) {
    this->pending.activate_preset = true;
    this->trigger_cycle();
  }

  this->pending.climate = settings;
}

/**
 * Trigger a query cycle
 *
 * If not active, kick off a cycle.
 * If already active, set a flag to run another when it finishes.
 */
void DaikinS21::trigger_cycle() {
  if (this->cycle_active == false) {
    start_cycle();
  } else {
    this->cycle_triggered = true;
  }
}

/**
 * Start a query cycle, tramsitting the first.
 */
void DaikinS21::start_cycle() {
  this->cycle_active = true;
  this->cycle_triggered = this->is_free_run();
  this->cycle_time_start_ms = millis();
  this->current_query = queries.begin();
  this->serial.send_frame(this->current_query->command);
}

/**
 * Check if a query is in the active pool and has been acked
 */
bool DaikinS21::is_query_active(std::string_view query_str) const {
  const auto query = std::ranges::find(this->queries, query_str, DaikinQueryState::GetCommand);
  return (query != this->queries.end()) && query->acked;
}

/**
 * Check if a query is in the unsupported pool (i.e. has been NAK'd)
 */
bool DaikinS21::is_query_unsupported(std::string_view query_str) const {
  return std::ranges::find(this->failed_queries, query_str) != this->failed_queries.end();
}

/**
 * Get a pointer to the results of a static query
 *
 * @return pointer to result buffer, nullptr if not resolved (yet)
 */
const PayloadBuffer* DaikinS21::get_static_query(std::string_view query_str) const {
  const auto iter = std::ranges::find(this->static_queries, query_str, DaikinQueryState::GetCommand);
  if (iter != this->static_queries.end()) {
    return &(iter->value);
  } else {
    return nullptr;
  }
}

/**
 * Remove a query from the active pool.
 *
 * Fixes up the current_query iterator.
 */
void DaikinS21::prune_query(std::string_view query_str) {
  const auto query = std::ranges::find(this->queries, query_str, DaikinQueryState::GetCommand);
  if (query != this->queries.end()) {
    // current_query iterator will be invalidated, recover index and recreate
    const auto query_index = std::distance(this->queries.begin(), query);
    const auto current_index = std::distance(this->queries.begin(), this->current_query);
    this->queries.erase(query);
    this->current_query = this->queries.begin() + current_index + ((current_index > query_index) ? -1 : 0);  // adjust index for the hole
  }
}

/**
 * Refine the pool of polling queries, adding or removing them as we learn about the unit.
 */
void DaikinS21::refine_queries() {
  if (this->protocol_version == ProtocolUndetected) {
    if (this->determine_protocol_version()) {
      ESP_LOGI(TAG, "Protocol version %" PRI_SV " detected", PRI_SV_ARGS(protocol_to_string(this->protocol_version)));
    }
  }
  if (this->ready.all() == false) {
    // Some units don't support more granular sensor queries
    if (this->ready[ReadySensorReadout] == false) {
      this->support.inside_temperature = this->is_query_active(EnvironmentQuery::InsideTemperature);
      this->support.outside_temperature = this->is_query_active(EnvironmentQuery::OutsideTemperature);
      if (this->support.inside_temperature && this->support.outside_temperature) {
        this->prune_query(StateQuery::InsideOutsideTemperatures);  // support for discrete granular sensors, no need for inferior consolidated query
      }
      // done if both queries have been resolved
      this->ready[ReadySensorReadout] = (this->support.inside_temperature || this->is_query_unsupported(EnvironmentQuery::InsideTemperature))
                                     && (this->support.outside_temperature || this->is_query_unsupported(EnvironmentQuery::OutsideTemperature));
    }
  }
}

void DaikinS21::send_command(std::string_view command, std::span<const uint8_t> payload) {
  this->current_command = command;
  this->serial.send_frame(this->current_command, payload);
}

/**
 * Perform the next action when the communication state is idle
 *
 * Usually selects the next command to issue to the unit
 *
 * Handle the results of the cycle if the last command was already issued:
 * - Process and report the results if the cycle was just completed
 * - Start the next cycle if free running or triggered in polling mode
 */
void DaikinS21::handle_serial_idle() {
  PayloadBuffer payload;

  // Apply any pending settings
  // Important to clear the activate flag here as another command can be queued while waiting for this one to complete
  if (this->pending.activate_climate) {
    this->pending.activate_climate = false;
    payload[0] = (this->pending.climate.mode == climate::CLIMATE_MODE_OFF) ? '0' : '1'; // power
    payload[1] = climate_mode_to_daikin(this->pending.climate.mode);
    payload[2] = (static_cast<int16_t>(this->pending.climate.setpoint) / 5) + 28;
    payload[3] = static_cast<char>(this->pending.climate.fan);
    this->send_command(StateCommand::PowerModeTempFan, payload);
    return;
  }

  if (this->pending.activate_swing_mode) {
    this->pending.activate_swing_mode = false;
    payload[0] = climate_swing_mode_to_daikin(this->pending.climate.swing);
    payload[1] = (this->pending.climate.swing != climate::CLIMATE_SWING_OFF) ? '?' : '0';
    payload[2] = '0';
    payload[3] = '0';
    this->send_command(StateCommand::LouvreSwingMode, payload);
    return;
  }

  if (this->pending.activate_preset) {
    // potentially a two stage operation -- first disabling the old, then enabling the new
    climate::ClimatePreset preset;
    bool enable;
    if ((this->current.climate.preset != this->pending.climate.preset) && (this->current.climate.preset != climate::CLIMATE_PRESET_NONE)) {
      preset = this->current.climate.preset;
      enable = false;
      if (this->pending.climate.preset == climate::CLIMATE_PRESET_NONE) {
        this->pending.activate_preset = false;  // don't execute again if we're just disabling the current preset
      }
    } else {
      preset = this->pending.climate.preset;
      enable = true;
      this->pending.activate_preset = false;  // don't execute again if we're setting the desired preset
    }
    switch (preset) {
      case climate::CLIMATE_PRESET_BOOST:
        payload[0] = enable ? '2' : '0';
        payload[1] = '0';
        payload[2] = '0';
        payload[3] = '0';
        this->send_command(StateCommand::Powerful, payload);
        return;
      case climate::CLIMATE_PRESET_ECO:
        payload[0] = '0';
        payload[1] = enable ? '2' : '0';
        payload[2] = '0';
        payload[3] = '0';
        this->send_command(StateCommand::Econo, payload);
        return;
      case climate::CLIMATE_PRESET_NONE:
      default:
        break;
    }
  }

  // Periodic query cycle
  if (this->current_query != this->queries.end()) {
    this->serial.send_frame(this->current_query->command);  // query cycle underway, continue
    return;
  }

  // Query cycle complete
  const auto now = millis();
  this->cycle_active = false;
  this->refine_queries();
  this->cycle_time_ms = now - this->cycle_time_start_ms;
  if (this->ready.all()) {
    if ((this->current.action_reported == climate::CLIMATE_ACTION_FAN) || this->unit_state.active()) {
      this->current.action = this->current.action_reported;
    } else {
      this->current.action = climate::CLIMATE_ACTION_IDLE;
    }
    if (this->modifiers[ModifierPowerful]) {
      this->current.climate.preset = climate::CLIMATE_PRESET_BOOST;
    } else if (this->modifiers[ModifierEcono]) {
      this->current.climate.preset = climate::CLIMATE_PRESET_ECO;
    } else {
      this->current.climate.preset = climate::CLIMATE_PRESET_NONE;
    }

    // signal there's fresh data
    if (this->binary_sensor_callback) {
      this->binary_sensor_callback(this->unit_state, this->system_state);
    }
    if (this->climate_callback) {
      this->climate_callback();
    }
  }

  if ((now - last_state_dump_ms) > (60 * 1000)) { // every minute
    last_state_dump_ms = now;
    this->enable_loop();  // dump state in foreground, blocks for too long here
  }

  // Start fresh polling query cycle (triggered never cleared in free run)
  if (this->cycle_triggered) {
    this->start_cycle();
  }
}

void DaikinS21::handle_state_basic(const std::span<const uint8_t> payload) {
  if (payload[0] == '0') {
    this->current.climate.mode = climate::CLIMATE_MODE_OFF;
    this->current.action_reported = climate::CLIMATE_ACTION_OFF;
  } else {
    this->current.climate.mode = daikin_to_climate_mode(payload[1]);
    this->current.action_reported = daikin_to_climate_action(payload[1]);
  }
  this->current.climate.setpoint = (payload[2] - 28) * 5;  // Celsius * 10
  // fan mode in payload[3], silent mode not reported so prefer RG
  this->ready.set(ReadyBasic);
}

void DaikinS21::handle_state_optional_features(const std::span<const uint8_t> payload) {
  if (payload[0] & 0b0100) {
    this->support.swing = true;
    this->queries.emplace_back(EnvironmentQuery::VerticalSwingAngle, &DaikinS21::handle_env_vertical_swing_angle);
  }
  this->support.horizontal_swing = payload[0] & 0b1000;
  // todo climate traits config check
  this->G2_model_info = (payload[1] & 0b1000) ? 'N': 'C';
  ESP_LOGI(TAG, "Capabilities detected: model info %c", this->G2_model_info);
  if (payload[3] & 0b0010) {
    this->support.humidity = true;
    this->queries.emplace_back(EnvironmentQuery::IndoorHumidity, &DaikinS21::handle_env_indoor_humidity);
  }
}

void DaikinS21::handle_state_swing_or_humidity(const std::span<const uint8_t> payload) {
  this->current.climate.swing = daikin_to_climate_swing_mode(payload[0]);
}

void DaikinS21::handle_state_special_modes(const std::span<const uint8_t> payload) {
  this->modifiers[ModifierPowerful] = (payload[0] & 0b00000010);
  this->modifiers[ModifierComfort] =  (payload[0] & 0b01000000);
  this->modifiers[ModifierQuiet] =    (payload[0] & 0b10000000);
  this->modifiers[ModifierStreamer] = (payload[1] & 0b10000000);
  this->modifiers[ModifierSensor] =   (payload[3] & 0b00001000);
  this->modifiers[ModifierLED] =      (payload[3] & 0b00001100) == 0b00001100;
}

void DaikinS21::handle_state_demand_and_econo(const std::span<const uint8_t> payload) {
  this->modifiers[ModifierEcono] =    (payload[1] == '2');
}

void DaikinS21::handle_state_inside_outside_temperature(const std::span<const uint8_t> payload) {
  if (this->support.inside_temperature == false) {  // more granular in EnvironmentQuery::InsideTemperature
    this->temp_inside = (payload[0] - 128) * 5;  // 1 degree
  }
  if (this->support.outside_temperature == false) { // more granular in EnvironmentQuery::OutsideTemperature
    this->temp_outside = (payload[1] - 128) * 5; // 1 degree, danijelt reports 0xFF when unsupported
  }
  if ((payload[2] - '0') <= 100) {  // Some units report 0xFF when unsupported
    this->humidity = payload[2] - '0';  // 5% granularity
  }
}

/** Vastly inferior to StateQuery::Basic */
void DaikinS21::handle_env_power_on_off(std::span<const uint8_t> payload) {
  const bool active = payload[0] == '1';
}

/** Same info as StateQuery::Basic */
void DaikinS21::handle_env_indoor_unit_mode(const std::span<const uint8_t> payload) {
  if (payload[0] == '0') {
    this->current.climate.mode = climate::CLIMATE_MODE_OFF;
    this->current.action_reported = climate::CLIMATE_ACTION_OFF;
  } else {
    this->current.climate.mode = daikin_to_climate_mode(payload[1]);
    this->current.action_reported = daikin_to_climate_action(payload[1]);
  }
}

/** Same info as StateQuery::Basic */
void DaikinS21::handle_env_temperature_setpoint(const std::span<const uint8_t> payload) {
  this->current.climate.setpoint = bytes_to_num(payload) * 10;  // whole degrees C
}

/** Same info as StateQuery::SwingOrHumidity */
void DaikinS21::handle_env_swing_mode(const std::span<const uint8_t> payload) {
  this->current.climate.swing = daikin_to_climate_swing_mode(payload[0]);
}

/** Better info than StateQuery::Basic (reports quiet) */
void DaikinS21::handle_env_fan_mode(const std::span<const uint8_t> payload) {
  this->current.climate.fan = static_cast<daikin_s21::DaikinFanMode>(payload[0]);
}

void DaikinS21::handle_env_inside_temperature(const std::span<const uint8_t> payload) {
  this->temp_inside = bytes_to_num(payload);
}

void DaikinS21::handle_env_liquid_temperature(const std::span<const uint8_t> payload) {
  this->temp_coil = bytes_to_num(payload);
}

void DaikinS21::handle_env_fan_speed_setpoint(std::span<const uint8_t> payload) {
  this->current.fan_rpm_setpoint = bytes_to_num(payload) * 10;
}

void DaikinS21::handle_env_fan_speed(const std::span<const uint8_t> payload) {
  this->current.fan_rpm = bytes_to_num(payload) * 10;
}

void DaikinS21::handle_env_vertical_swing_angle_setpoint(std::span<const uint8_t> payload) {
  this->current.swing_vertical_angle_setpoint = bytes_to_num(payload);
}

void DaikinS21::handle_env_vertical_swing_angle(const std::span<const uint8_t> payload) {
  this->current.swing_vertical_angle = bytes_to_num(payload);
}

void DaikinS21::handle_env_target_temperature(const std::span<const uint8_t> payload) {
  this->temp_target = bytes_to_num(payload); // Internal control loop target temperature
}

void DaikinS21::handle_env_outside_temperature(const std::span<const uint8_t> payload) {
  this->temp_outside = bytes_to_num(payload);
}

void DaikinS21::handle_env_indoor_frequency_command_signal(const std::span<const uint8_t> payload) {
  this->demand = bytes_to_num(payload);  // Demand, 0-15
}

void DaikinS21::handle_env_compressor_frequency(const std::span<const uint8_t> payload) {
  this->compressor_hz = bytes_to_num(payload);
  if (this->compressor_hz == 999) {
    this->compressor_hz = 0;  // reported by danijelt
  }
}

void DaikinS21::handle_env_indoor_humidity(const std::span<const uint8_t> payload) {
  this->humidity = bytes_to_num(payload);
}

void DaikinS21::handle_env_unit_state(const std::span<const uint8_t> payload) {
  this->unit_state = ahex_digit(payload[0]);
  this->modifiers[ModifierPowerful] = this->unit_state.powerful();  // if G6 is unsupported we can still read out powerful set by remote
}

void DaikinS21::handle_env_system_state(const std::span<const uint8_t> payload) {
  this->system_state = ahex_u8_le(payload[0], payload[1]);
}

/**
 * Determine the protocol version according to Faikin documentation.
 *
 * @note This is mostly untested, I own only one Daikin system.
 *
 * @return true protocl version was detected
 * @return false protocol version wasn't detected
 */
bool DaikinS21::determine_protocol_version() {
  static constexpr uint8_t old_version_0[4] = {'0',0,0,0};
  static constexpr uint8_t old_version_2or3[4] = {'0','2',0,0};
  static constexpr uint8_t old_version_31plus[4] = {'0','2','0','0'};
  static constexpr uint8_t new_version_300[4] = {'0','0','3','0'};
  static constexpr uint8_t new_version_320[4] = {'0','2','3','0'};
  static constexpr uint8_t new_version_340[4] = {'0','4','3','0'};

  // both protocol indicators should have been resolved if we're in here
  const auto old_proto = this->get_static_query(StateQuery::OldProtocol);
  const bool old_failed = this->is_query_unsupported(StateQuery::OldProtocol);
  const auto new_proto = this->get_static_query(StateQuery::NewProtocol);
  const bool new_failed = this->is_query_unsupported(StateQuery::NewProtocol);

  // Check availability first
  if (old_proto && new_failed) {
    if (std::ranges::equal(*old_proto, old_version_0)) {
      this->protocol_version = Protocol0;
    } else if (std::ranges::equal(*old_proto, old_version_2or3) || std::ranges::equal(*old_proto, old_version_31plus)) {
      this->protocol_version = Protocol2; // NAK for NewProtocol rules out 3.0
    } else {
      this->protocol_version = ProtocolUnknown;
    }
  } else if ((old_proto || old_failed) && new_proto) {
    if (std::ranges::equal(*new_proto, new_version_300)) {
      if (old_failed) {
        this->protocol_version = ProtocolUnknown; // Need old protocol to make the distinction
      } else if (std::ranges::equal(*old_proto, old_version_2or3)) {
        this->protocol_version = Protocol3_0; // ACK for NewProtocol means 3.0 has support for this query
      } else if (std::ranges::equal(*old_proto, old_version_31plus)) {
        this->protocol_version = Protocol3_1;
      } else {
        this->protocol_version = ProtocolUnknown;
      }
    } else if (std::ranges::equal(*new_proto, new_version_320)) {
      this->protocol_version = Protocol3_2;
    } else if (std::ranges::equal(*new_proto, new_version_340)) {
      this->protocol_version = Protocol3_4;
    } else {
      this->protocol_version = ProtocolUnknown;
    }
  } else if (old_failed && new_failed) {
    // both nak'd, even though we're talking to the unit
    ESP_LOGE(TAG, "Unable to detect a protocol version!");
    this->protocol_version = ProtocolUnknown;
  } else {
    ESP_LOGV(TAG, "Protocol version not ready yet");
  }

  // Print some info if we're falling back to version 0
  if (this->protocol_version == ProtocolUnknown) {
    ESP_LOGE(TAG, "Unable to detect protocol version! Old: %s New: %s",
      old_proto ? str_repr(*old_proto).c_str() : "NAK",
      new_proto ? str_repr(*new_proto).c_str() : "NAK");
  }

  return this->protocol_version != daikin_s21::ProtocolUndetected;
}

void DaikinS21::handle_serial_result(const DaikinSerial::Result result, const std::span<const uint8_t> response /*= {}*/) {
  const bool is_query = this->current_command.empty();
  const std::string_view &tx_str = is_query ? this->current_query->command : this->current_command;
  std::span<const uint8_t> payload = { response.begin() + tx_str.size(), response.end() };

  // add commands to this array to debug their output. empty string is just a placeholder to compile
  static constexpr std::array debug_commands{""};
  const bool is_debug = this->debug && (std::ranges::find(debug_commands, tx_str) != debug_commands.end());

  // debug logging
  switch (result) {
    case DaikinSerial::Result::Ack:
      if (/*this->debug ||*/ is_debug) {  // uncomment to debug all, not just debug_commands
        ESP_LOGD(TAG, "ACK: %" PRI_SV " -> %s %s",
                  PRI_SV_ARGS(tx_str),
                  str_repr(payload).c_str(),
                  hex_repr(payload).c_str());
      }
      break;
    case DaikinSerial::Result::Nak:
      ESP_LOGW(TAG, "NAK for %" PRI_SV, PRI_SV_ARGS(tx_str));
      break;
    case DaikinSerial::Result::Timeout:
      ESP_LOGW(TAG, "Timeout waiting for response to %" PRI_SV, PRI_SV_ARGS(tx_str));
      break;
    case DaikinSerial::Result::Error:
      ESP_LOGE(TAG, "Error with %" PRI_SV, PRI_SV_ARGS(tx_str));
      break;
    default:
      break;
  }

  // handle serial result
  switch (result) {
    case DaikinSerial::Result::Ack:
      this->comms_detected = true;
      if (is_query) {
        const auto common_length = std::min(payload.size(), this->current_query->value.size());
        // mark as acked
        this->current_query->acked = true;
        this->current_query->naks = 0;
        // print changed values
        if ((/*this->debug ||*/ is_debug) &&  // uncomment to debug all, not just debug_commands
            (std::ranges::equal(this->current_query->value.begin(), this->current_query->value.begin() + common_length, payload.begin(), payload.begin() + common_length) == false)) {
          ESP_LOGI(TAG, "%" PRI_SV " changed: %s %s -> %s %s",
                    PRI_SV_ARGS(this->current_query->command),
                    str_repr(this->current_query->value).c_str(),
                    hex_repr(this->current_query->value).c_str(),
                    str_repr(payload).c_str(),
                    hex_repr(payload).c_str());
        }
        // save a copy of the payload
        std::copy_n(payload.begin(), common_length, this->current_query->value.begin());
        if (this->current_query->is_static) {
          this->static_queries.emplace_back(*this->current_query);
        }
        // decode payload
        if (this->current_query->handler != nullptr) {
          std::invoke(this->current_query->handler, this, payload);
        } else {
          ESP_LOGI(TAG, "Unhandled command: %s", hex_repr(response).c_str());
        }
      } else {
        // nothing yet to do when a command is accepted
      }
      break;

    case DaikinSerial::Result::Nak:
      this->comms_detected = true;
      if (is_query) {
        this->current_query->naks++;
      } else {
        // nothing yet to do when a command is rejected
      }
      break;

    case DaikinSerial::Result::Timeout:
      if (is_query) {
        // It's possible some unsupported queries don't respond at all
        // Treat these as NAKs if we've established communication
        // Otherwise, a disconnected or unpowered HVAC unit will quickly cause all queries to fail
        if (this->comms_detected) {
          this->current_query->naks++;
        }
      } else {
        // command will be retried, let's hope the code that generates it is bug free
      }
      break;

    case DaikinSerial::Result::Error:
    default:
      break;
  }

  // update local state for next action
  if (result == DaikinSerial::Result::Error) {
    // something went terribly wrong, try to reinitialize communications
    this->pending.activate_climate = false;
    this->pending.activate_swing_mode = false;
    this->pending.activate_preset = false;
    this->current_command = {};
    this->start_cycle();
  } else {
    if (is_query) {
      if ((result == DaikinSerial::Result::Ack) && this->current_query->is_static) {
        // got the query result and know it won't change, remove
        this->prune_query(this->current_query->command);
      } else if (this->current_query->naks >= 3) {
        // query failed, remove
        ESP_LOGW(TAG, "removing %" PRI_SV " from query pool as unsupported", PRI_SV_ARGS(tx_str));
        this->failed_queries.emplace_back(this->current_query->command);
        this->prune_query(this->current_query->command);
      } else {
        // next query
        this->current_query++;
      }
      // if all queries have failed we probably had comms then they were lost
      if (this->queries.empty()) {
        this->setup();  // reinitialize in order to prepare for the HVAC unit being reconnected
      }
    } else {
      this->current_command = {};
    }
  }
}

void DaikinS21::dump_state() {
  ESP_LOGD(TAG, "  Connected: %s  Protocol: %" PRI_SV,
          this->comms_detected ? "true" : "false",
          PRI_SV_ARGS(protocol_to_string(this->protocol_version)));
  if (this->debug) {
    const auto old_proto = this->get_static_query(StateQuery::OldProtocol);
    const auto new_proto = this->get_static_query(StateQuery::NewProtocol);
    const auto model_code = this->get_static_query(StateQuery::ModelCode);
    const auto misc_model = this->get_static_query(MiscQuery::Model);
    const auto misc_version = this->get_static_query(MiscQuery::Version);
    ESP_LOGD(TAG, "      G8: %s  GC: %s  GY00: %s  M: %s  V: %s",
      old_proto ? str_repr(*old_proto).c_str() : "N/A",
      new_proto ? str_repr(*new_proto).c_str() : "N/A",
      model_code ? str_repr(*model_code).c_str() : "N/A",
      misc_model ? str_repr(*misc_model).c_str() : "N/A",
      misc_version ? str_repr(*misc_version).c_str() : "N/A");
  }
  ESP_LOGD(TAG, "   Mode: %s  Action: %s  Preset: %s",
          LOG_STR_ARG(climate::climate_mode_to_string(this->current.climate.mode)),
          LOG_STR_ARG(climate::climate_action_to_string(this->get_climate_action())),
          LOG_STR_ARG(climate::climate_preset_to_string(this->current.climate.preset)));
  ESP_LOGD(TAG, "    Fan: %" PRI_SV " (%" PRIu16 " RPM)  Swing: %s",
          PRI_SV_ARGS(daikin_fan_mode_to_string_view(this->current.climate.fan)), this->current.fan_rpm,
          (this->support.swing ? LOG_STR_ARG(climate::climate_swing_mode_to_string(this->current.climate.swing)) : "unsupported"));
  ESP_LOGD(TAG, " Target: %.1f C (%.1f F)",
          this->current.climate.setpoint.f_degc(), this->current.climate.setpoint.f_degf());
  ESP_LOGD(TAG, " Inside: %.1f C (%.1f F)",
          this->temp_inside.f_degc(), this->temp_inside.f_degf());
  ESP_LOGD(TAG, "Outside: %.1f C (%.1f F)",
          this->temp_outside.f_degc(), this->temp_outside.f_degf());
  ESP_LOGD(TAG, "   Coil: %.1f C (%.1f F)",
          this->temp_coil.f_degc(), this->temp_coil.f_degf());
  if (this->support.humidity) {
    ESP_LOGD(TAG, "  Humid: %" PRIu8 "%%", this->get_humidity());
  }
  ESP_LOGD(TAG, " Demand: %" PRIu8, this->get_demand());
  ESP_LOGD(TAG, " Cycle Time: %" PRIu32 "ms", this->cycle_time_ms);
  ESP_LOGD(TAG, " UnitState: %" PRIX8 " SysState: %02" PRIX8, this->unit_state.raw, this->system_state.raw);
  if (this->debug) {
    const auto comma_join = [](const auto& queries) {
      std::string str;
      for (const auto &q : queries) {
        str += q;
        if (q != queries.back()) {
          str += ",";
        }
      }
      return str;
    };
    ESP_LOGD(TAG, LOG_STR_ARG((" Active: " + comma_join(std::views::transform(this->queries, DaikinQueryState::GetCommand))).c_str()));
    ESP_LOGD(TAG, LOG_STR_ARG(("  Nak'd: " + comma_join(this->failed_queries)).c_str()));
    ESP_LOGD(TAG, LOG_STR_ARG((" Static: " + comma_join(std::views::transform(this->static_queries, DaikinQueryState::GetCommand))).c_str()));
  }
}

} // namespace esphome::daikin_s21
