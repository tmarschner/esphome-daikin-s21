#pragma once

#include <bitset>
#include <limits>
#include <vector>
#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "daikin_s21_fan_modes.h"

namespace esphome {
namespace daikin_s21 {

class DaikinSerial {
public:
  static constexpr uint32_t S21_MAX_COMMAND_SIZE = 4;
  static constexpr uint32_t S21_PAYLOAD_SIZE = 4;

  enum class Result : uint8_t {
    Idle,
    Ack,
    Nak,
    Error,
    Timeout,
    Busy,
  };
  
  DaikinSerial() {};
  DaikinSerial(uart::UARTComponent *tx, uart::UARTComponent *rx);
  
  Result service();
  Result send_frame(const char *cmd, const std::array<char, S21_PAYLOAD_SIZE> *payload = nullptr);
  void flush_input();
  
  std::vector<uint8_t> response = {};
  bool debug = false;

private:
  Result handle_rx(uint8_t byte);

  enum class CommState : uint8_t {
    Idle,
    CommandAck,
    QueryAck,
    QueryStx,
    QueryEtx,
    AckResponseDelay,
    NextTxDelay,
    ErrorDelay,
  };

  uart::UARTComponent *tx_uart{nullptr};
  uart::UARTComponent *rx_uart{nullptr};
  CommState comm_state = CommState::Idle;
  uint32_t last_event_time_ms = 0;
};


struct DaikinSettings {
  climate::ClimateMode mode = climate::CLIMATE_MODE_OFF;
  int16_t setpoint = 23 * 10;
  DaikinFanMode fan = DaikinFanMode::Auto;
  climate::ClimateSwingMode swing = climate::CLIMATE_SWING_OFF;
};


class DaikinS21 : public PollingComponent {
 public:
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;

  void set_uarts(uart::UARTComponent *tx, uart::UARTComponent *rx) { this->serial = {tx, rx}; }
  void set_debug_comms(bool set) { this->serial.debug = set; }
  void set_debug_protocol(bool set) { this->debug_protocol = set; }

  // external command actions
  void set_climate_settings(climate::ClimateMode mode, float setpoint, DaikinFanMode fan_mode);
  void set_swing_settings(climate::ClimateSwingMode swing);

  bool is_ready() { return this->ready.all(); }
  climate::ClimateMode get_climate_mode() { return this->active.mode; }
  climate::ClimateAction get_climate_action();
  DaikinFanMode get_fan_mode() { return this->active.fan; }
  climate::ClimateSwingMode get_swing_mode() { return this->active.swing; }
  float get_setpoint() { return this->active.setpoint / 10.0F; }
  float get_temp_inside() { return this->temp_inside / 10.0; }
  float get_temp_outside() { return this->temp_outside / 10.0; }
  float get_temp_coil() { return this->temp_coil / 10.0; }
  auto get_fan_rpm() { return this->fan_rpm; }
  auto get_swing_vertical_angle() { return this->swing_vertical_angle; }
  auto get_compressor_frequency() { return this->compressor_hz; }
  auto get_humidity() { return this->humidity; }
  auto get_demand() { return this->demand; }

 protected:
  DaikinSerial serial;

  void dump_state();
  void refine_queries();
  void tx_next();
  void parse_ack();
  void handle_nak();

  enum ReadyCommand : uint8_t {
    ReadyProtocolVersion,
    ReadyCapabilities,
    ReadyBasic,
    ReadyCount, // just for bitset sizing
  };
  std::bitset<ReadyCount> ready = {};

  // communication state
  bool is_query_active(const char * query_str);
  bool prune_query(const char * query_str);
  std::vector<const char *> queries = {};
  std::vector<const char *>::iterator current_query;
  const char *tx_command = "";  // used when matching responses - value must have persistent lifetime across serial state machine runs
  bool refresh_state = false;
  bool debug_protocol = false;
  std::unordered_map<std::string, std::vector<uint8_t>> val_cache;  // debugging

  // settings
  DaikinSettings active = {};
  DaikinSettings pending = {};
  bool activate_climate = false;
  bool activate_swing_mode = false;

  // current values
  climate::ClimateAction climate_action = climate::CLIMATE_ACTION_OFF;
  int16_t temp_inside = 0;
  int16_t temp_target = 0;
  int16_t temp_outside = 0;
  int16_t temp_coil = 0;
  uint16_t fan_rpm = 0;
  int16_t swing_vertical_angle = 0;
  uint8_t compressor_hz = 0;
  uint8_t humidity = 50;
  uint8_t demand = 0;

  // protocol support
  bool determine_protocol_version();
  uint8_t G8[4] = {};
  uint16_t GY00 = 0;
  struct ProtocolVersion {
    uint8_t major = std::numeric_limits<uint8_t>::max();
    uint8_t minor = std::numeric_limits<uint8_t>::max();
  };
  ProtocolVersion protocol_version = {};
  char G2_model_info = 0;
  bool support_swing = false;
  bool support_horizontal_swing = false;
  bool support_humidity = false;
};

class DaikinS21Client {
 public:
  void set_s21(DaikinS21 *s21) { this->s21 = s21; }

 protected:
  DaikinS21 *s21;
};

}  // namespace daikin_s21
}  // namespace esphome
