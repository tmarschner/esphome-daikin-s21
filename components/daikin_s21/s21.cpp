#include <algorithm>
#include <cinttypes>
#include <numeric>
#include "s21.h"

using namespace esphome;

namespace esphome {
namespace daikin_s21 {

#define STX 2
#define ETX 3
#define ENQ 5
#define ACK 6
#define NAK 21

static const char *const TAG = "daikin_s21";

uint8_t climate_mode_to_daikin(const climate::ClimateMode mode) {
  switch (mode) {
    case climate::CLIMATE_MODE_OFF:
    case climate::CLIMATE_MODE_HEAT_COOL:
    case climate::CLIMATE_MODE_AUTO:
    default:
      return '1';
      break;
    case climate::CLIMATE_MODE_HEAT:
      return '4';
    case climate::CLIMATE_MODE_COOL:
      return '3';
    case climate::CLIMATE_MODE_FAN_ONLY:
      return '6';
    case climate::CLIMATE_MODE_DRY:
      return '2';    
  }
}

climate::ClimateMode daikin_to_climate_mode(const uint8_t mode) {
  switch (mode) {
    case '0': // heat_cool cooling
    case '1': // heat_cool setting
    case '7': // heat_cool heating
    default:
      return climate::CLIMATE_MODE_HEAT_COOL;
    case '2': // dry
      return climate::CLIMATE_MODE_DRY;
    case '3': // cool
      return climate::CLIMATE_MODE_COOL;
    case '4': // heat
      return climate::CLIMATE_MODE_HEAT;
    case '6': // fan_only
      return climate::CLIMATE_MODE_FAN_ONLY;      
  }
}

climate::ClimateAction daikin_to_climate_action(const uint8_t action) {
  switch (action) {
    case '1': // heat_cool
    default:
      return climate::CLIMATE_ACTION_IDLE;
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
  }
}

std::string daikin_fan_mode_to_string(const DaikinFanMode mode) {
  switch (mode) {
    case DaikinFanMode::Auto:
      return "Auto";
    case DaikinFanMode::Silent:
      return "Silent";
    case DaikinFanMode::Speed1:
      return "1";
    case DaikinFanMode::Speed2:
      return "2";
    case DaikinFanMode::Speed3:
      return "3";
    case DaikinFanMode::Speed4:
      return "4";
    case DaikinFanMode::Speed5:
      return "5";
    default:
      return "UNKNOWN";
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
    case climate::CLIMATE_SWING_OFF:
    default:
      return '0';
    case climate::CLIMATE_SWING_BOTH:
      return '7';
    case climate::CLIMATE_SWING_VERTICAL:
      return '1';
    case climate::CLIMATE_SWING_HORIZONTAL:
      return '2';
  }
}

int16_t bytes_to_num(uint8_t *bytes, size_t len) {
  // <ones><tens><hundreds><neg/pos>
  int16_t val = 0;
  val = bytes[0] - '0';
  val += (bytes[1] - '0') * 10;
  val += (bytes[2] - '0') * 100;
  if (len > 3 && bytes[3] == '-')
    val *= -1;
  return val;
}

int16_t temp_bytes_to_c10(uint8_t *bytes) { return bytes_to_num(bytes, 4); }

int16_t temp_f9_byte_to_c10(uint8_t byte) { return (byte - 128) * 5; }

uint8_t c10_to_setpoint_byte(int16_t setpoint) {
  return (setpoint + 3) / 5 + 28;
}

#define S21_BAUD_RATE 2400
#define S21_STOP_BITS 2
#define S21_DATA_BITS 8
#define S21_PARITY uart::UART_CONFIG_PARITY_EVEN

void DaikinSerial::set_uarts(uart::UARTComponent *tx, uart::UARTComponent *rx) {
  this->tx_uart = tx;
  this->rx_uart = rx;

  for (auto *uart : {this->tx_uart, this->rx_uart}) {
    uart->set_baud_rate(S21_BAUD_RATE);
    uart->set_stop_bits(S21_STOP_BITS);
    uart->set_data_bits(S21_DATA_BITS);
    uart->set_parity(S21_PARITY);
    uart->load_settings();
  }
}

void DaikinS21::dump_config() {
  ESP_LOGCONFIG(TAG, "DaikinS21:");
  ESP_LOGCONFIG(TAG, "  Update interval: %" PRIu32, this->get_update_interval());
}

// Adapated from ESPHome UART debugger
std::string hex_repr(const uint8_t *bytes, size_t len) {
  std::string res;
  char buf[5];
  for (size_t i = 0; i < len; i++) {
    if (i > 0)
      res += ':';
    sprintf(buf, "%02X", bytes[i]);
    res += buf;
  }
  return res;
}

// Adapated from ESPHome UART debugger
std::string str_repr(const uint8_t *bytes, size_t len) {
  std::string res;
  char buf[5];
  for (size_t i = 0; i < len; i++) {
    if (bytes[i] == 7) {
      res += "\\a";
    } else if (bytes[i] == 8) {
      res += "\\b";
    } else if (bytes[i] == 9) {
      res += "\\t";
    } else if (bytes[i] == 10) {
      res += "\\n";
    } else if (bytes[i] == 11) {
      res += "\\v";
    } else if (bytes[i] == 12) {
      res += "\\f";
    } else if (bytes[i] == 13) {
      res += "\\r";
    } else if (bytes[i] == 27) {
      res += "\\e";
    } else if (bytes[i] == 34) {
      res += "\\\"";
    } else if (bytes[i] == 39) {
      res += "\\'";
    } else if (bytes[i] == 92) {
      res += "\\\\";
    } else if (bytes[i] < 32 || bytes[i] > 127) {
      sprintf(buf, "\\x%02X", bytes[i]);
      res += buf;
    } else {
      res += bytes[i];
    }
  }
  return res;
}

DaikinSerial::Result DaikinSerial::handle_rx(const uint8_t byte) {
  Result result = Result::Busy; // default to busy, override when rx phase is over

  switch (comm_state) {
    case CommState::QueryAck:
    case CommState::CommandAck:
      switch (byte) {
        case ACK:
          if (comm_state == CommState::QueryAck) {
            comm_state = CommState::QueryStx;
          } else {
            comm_state = CommState::Cooldown;
            cooldown_length = S21_RESPONSE_TURNAROUND;
            result = Result::Ack;
          }
          break;
        case NAK:
          comm_state = CommState::Cooldown;
          cooldown_length = S21_RESPONSE_TURNAROUND;
          result = Result::Nak;
          break;
        default:
          ESP_LOGW(TAG, "Rx ACK: Unuexpected 0x%02X", byte);
          comm_state = CommState::Cooldown;
          cooldown_length = S21_ERROR_TIMEOUT;
          result = Result::Error;
          break;
      }
      break;

    case CommState::QueryStx:
      if (byte == STX) {
        comm_state = CommState::QueryEtx;
      } else if (byte == ACK) {
        ESP_LOGD(TAG, "Rx STX: Unuexpected extra ACK, ignoring"); // on rare occasions my unit will do this
      } else {
        ESP_LOGW(TAG, "Rx STX: Unuexpected 0x%02X", byte);
        comm_state = CommState::Cooldown;
        cooldown_length = S21_ERROR_TIMEOUT;
        result = Result::Error;
      }
      break;

    case CommState::QueryEtx:
      if (byte != ETX) {
        // not the end, add to buffer
        response.push_back(byte);
        if (response.size() > (S21_MAX_COMMAND_SIZE + S21_PAYLOAD_SIZE + 1)) {  // +1 for checksum byte
          ESP_LOGW(TAG, "Rx ETX: Overflow %s %s + 0x%02X",
            str_repr(response.data(), response.size()).c_str(),
            hex_repr(response.data(), response.size()).c_str(),
            byte);
          comm_state = CommState::Cooldown;
          cooldown_length = S21_ERROR_TIMEOUT;
          result = Result::Error;
        }
      } else {
        // frame received, validate checksum
        const uint8_t checksum = response[response.size() - 1];
        response.pop_back();
        const uint8_t calc_checksum = std::accumulate(response.begin(), response.end(), 0U);
        if ((calc_checksum == checksum)
            || ((calc_checksum == STX) && (checksum == ENQ))) {  // protocol avoids STX in message body
          tx_uart->write_byte(ACK);
          comm_state = CommState::Cooldown;
          cooldown_length = S21_RESPONSE_TURNAROUND;
          result = Result::Ack;
        } else {
          ESP_LOGW(TAG, "Rx ETX: Checksum mismatch: 0x%02X != 0x%02X (calc from %s)",
            checksum, calc_checksum, hex_repr(response.data(), response.size()).c_str());
          comm_state = CommState::Cooldown;
          cooldown_length = S21_ERROR_TIMEOUT;
          result = Result::Error;
        }
      }
      break;
      
    default:
      break;
  }

  return result;
}

DaikinSerial::Result DaikinSerial::service() {
  Result result = Result::Busy;

  switch(comm_state) {
  case CommState::Idle:
    result = Result::Idle;
    break;

  case CommState::Cooldown:
    if ((millis() - last_event_time) > cooldown_length) {
      comm_state = CommState::Idle;
      result = Result::Idle;
    }
    break;

  default:
    // all other states are actively receiving data from the unit
    if ((millis() - last_event_time) > S21_RESPONSE_TIMEOUT) {  // timed out?
      comm_state = CommState::Idle;
      result = Result::Timeout;
      break;
    }
    while ((result == Result::Busy) && rx_uart->available()) {  // read all available bytes
      uint8_t byte;
      rx_uart->read_byte(&byte);
      last_event_time = millis();
      result = handle_rx(byte);
    }
    break;
  }

  return result;
}

DaikinSerial::Result DaikinSerial::send_frame(const char *cmd, const std::array<char, S21_PAYLOAD_SIZE> *payload) {
  if (comm_state != CommState::Idle) {
    return Result::Busy;
  }

  size_t cmd_len = strlen(cmd);
  if (cmd_len > S21_MAX_COMMAND_SIZE) {
    ESP_LOGW(TAG, "Tx: Command '%s' too large", cmd);
    return Result::Error;
  }

  if (debug) {
    if (payload == nullptr) {
      ESP_LOGD(TAG, "Tx: %s", cmd);
    } else {
      ESP_LOGD(TAG, "Tx: %s %s %s", cmd,
               str_repr(reinterpret_cast<const uint8_t*>(payload->data()), payload->size()).c_str(),
               hex_repr(reinterpret_cast<const uint8_t*>(payload->data()), payload->size()).c_str());
    }
  }

  // prepare for response
  response.clear();
  flush_input();

  // transmit
  tx_uart->write_byte(STX);
  tx_uart->write_array(reinterpret_cast<const uint8_t *>(cmd), cmd_len);
  uint8_t checksum = std::accumulate(cmd, cmd + cmd_len, 0U);
  if (payload) {
    tx_uart->write_array(reinterpret_cast<const uint8_t *>(payload->data()), payload->size());
    checksum = std::accumulate(payload->data(), payload->data() + payload->size(), checksum);
  }
  if (checksum == STX) {
    checksum = ENQ;  // mid-message STX characters are escaped
  }
  tx_uart->write_byte(checksum);
  tx_uart->write_byte(ETX);

  // wait for result
  last_event_time = millis();
  comm_state = (payload != nullptr) ? CommState::CommandAck : CommState::QueryAck;

  return Result::Ack;
}

void DaikinSerial::flush_input() {  // would be a nice ESPHome API improvement
  uint8_t byte;
  while (rx_uart->available()) {
    rx_uart->read_byte(&byte);
  }
}

bool DaikinS21::is_query_active(const char * const query_str) {
  return std::find_if(std::begin(queries), std::end(queries), [=](const char * query){ return strcmp(query_str, query) == 0; }) != std::end(queries);
}

bool DaikinS21::prune_query(const char * const query_str) {
  const auto it = std::find_if(std::begin(queries), std::end(queries), [=](const char * query){ return strcmp(query_str, query) == 0; });
  if (it != std::end(queries)) {
    queries.erase(it);
    return true;
  } else {
    return false;
  }
}

void DaikinS21::refine_queries() {
  if (this->ready[ReadyProtocolVersion] == false) {
    if (determine_protocol_version()) {
      prune_query("F8");
      prune_query("GY00");
      ESP_LOGI(TAG, "Protocol version %u.%u detected", this->protocol_version.major, this->protocol_version.minor);
      this->ready.set(ReadyProtocolVersion);
    }
  }
}

void DaikinS21::tx_next() {
  std::array<char, DaikinSerial::S21_PAYLOAD_SIZE> payload;

  // select next command / query
  if (activate_climate) {
    tx_command = "D1";
    payload[0] = (pending.mode == climate::CLIMATE_MODE_OFF) ? '0' : '1'; // power
    payload[1] = climate_mode_to_daikin(pending.mode);
    payload[2] = c10_to_setpoint_byte(lroundf(round(pending.setpoint * 2) / 2 * 10.0));
    payload[3] = static_cast<char>(pending.fan);
    this->serial.send_frame(tx_command, &payload);
    return;
  }

  if (activate_swing_mode) {
    tx_command = "D5";
    payload[0] = climate_swing_mode_to_daikin(pending.swing);
    payload[1] = (pending.swing != climate::CLIMATE_SWING_OFF) ? '?' : '0';
    payload[2] = '0';
    payload[3] = '0';
    this->serial.send_frame(tx_command, &payload);
    return;
  }
  
  // periodic queries
  if (current_query != queries.end()) {
    tx_command = *current_query;  // query scan underway, continue
    this->serial.send_frame(tx_command);
    return;
  }
  
  // start fresh query scan (only after current scan is complete)
  if (refresh_state) {
    refresh_state = false;
    refine_queries();
    current_query = queries.begin(); 
    tx_command = *current_query;
    this->serial.send_frame(tx_command);
    return;
  }
}

void DaikinS21::parse_ack() {
  char rcode[DaikinSerial::S21_MAX_COMMAND_SIZE + 1] = {};
  uint8_t payload[DaikinSerial::S21_PAYLOAD_SIZE];
  const size_t rcode_len = strlen(tx_command);
  size_t payload_len = 0;

  ESP_LOGV(TAG, "Rx: ACK from S21 for command %s", tx_command);

  // prepare response buffers for decoding
  if (serial.response.empty()) {
    payload_len = 0;
    // commands don't return anything except an Ack, pretend we received the command itself to provide something to distinguish handling below
    std::copy_n(tx_command, rcode_len, rcode);
  } else {
    payload_len = serial.response.size() - rcode_len;
    std::copy_n(serial.response.data(), rcode_len, rcode);
    std::copy_n(serial.response.data() + rcode_len, payload_len, payload);
    // query successful, move to the next one
    current_query++;
  }

  switch (rcode[0]) {
    case 'G':  // F -> G
      switch (rcode[1]) {
        case '1':  // F1 -> G1 Basic State
          if (payload[0] == '0') {
            this->active.mode = climate::CLIMATE_MODE_OFF;
            this->climate_action = climate::CLIMATE_ACTION_OFF;
          } else {
            this->active.mode = daikin_to_climate_mode(payload[1]);
            this->climate_action = daikin_to_climate_action(payload[1]);
          }
          this->active.setpoint = ((payload[2] - 28) * 5);  // Celsius * 10
          // silent mode not reported in payload[3], prefer RG
          this->ready.set(ReadyBasic);
          return;
        case '5':  // F5 -> G5 -- Swing state
          this->active.swing = daikin_to_climate_swing_mode(payload[0]);
          return;
        case '8':  // F8 -> G8 -- Original protocol version
          std::copy_n(payload, payload_len, this->G8);
          return;
        case '9':  // F9 -> G9 -- Temperature, better support in RH and Ra (0.5 degree granularity)
          this->temp_inside = temp_f9_byte_to_c10(payload[0]);
          this->temp_outside = temp_f9_byte_to_c10(payload[1]);
          return;
        case 'Y':
          if ((rcode[2] == '0') && (rcode[3] == '0')) { // FY00 -> GY00 Newer protocol version
            this->GY00 = bytes_to_num(payload, payload_len);
          }
          return;
        default:
          break;
      }
      break;

    case 'S':  // R -> S
      switch (rcode[1]) {
        case 'B':  // Operational mode, single character, same info as G1
          return;
        case 'C':  // Setpoint, same info as G1
          this->active.setpoint = temp_bytes_to_c10(payload);
          return;
        case 'F':  // Swing mode, same info as G5. ascii hex string: 2F herizontal 1F vertical 7F both 00 off
          return;
        case 'G':  // Fan mode, better detail than G1 (reports quiet mode)
          this->active.fan = static_cast<daikin_s21::DaikinFanMode>(payload[0]);
          return;
        case 'H':  // Inside temperature
          this->temp_inside = temp_bytes_to_c10(payload);
          return;
        case 'I':  // Coil temperature
          this->temp_coil = temp_bytes_to_c10(payload);
          return;
        case 'L':  // Fan speed
          this->fan_rpm = bytes_to_num(payload, payload_len) * 10;
          return;
        case 'M':  // v_swing setpoint
          break;
        case 'N':  // Vertical swing angle
          this->swing_vertical_angle = bytes_to_num(payload, 4);
          return;
        case 'X':  // Internal control loop target temperature
          this->temp_target = temp_bytes_to_c10(payload);
          return;
        case 'a':  // Outside temperature
          this->temp_outside = temp_bytes_to_c10(payload);
          return;
        case 'b':  // Demand, 0-15
          this->demand = temp_bytes_to_c10(payload);
          return;
        case 'd':  // Compressor frequency in hertz, idle if 0.
          this->compressor_hz = bytes_to_num(payload, payload_len);
          return;
        case 'e':  // Humidity, %
          this->humidity = bytes_to_num(payload, payload_len);
          return;
        default:
          if (payload_len > 3) {
            int8_t temp = temp_bytes_to_c10(payload);
            ESP_LOGD(TAG, "Unknown sensor: %s -> %s -> %.1f C (%.1f F)", rcode,
                     hex_repr(payload, payload_len).c_str(), c10_c(temp),
                     c10_f(temp));
          }
          break;
      }
      break;

    case 'M':  // todo faikin says 100WH units of power
      break;

    case 'D':  // D -> D (fake response, see above)
      switch (rcode[1]) {
        case '1':
          this->activate_climate = false;
          break;
        case '5':
          this->activate_swing_mode = false;
          break;
        default:
          break;
      }
      this->refresh_state = true; // a command took, trigger immediate refresh
      return;

    default:
      break;
  }

  // protocol decoding debug
  // note: well known responses return directly from the switch statements
  // break instead if you want to view their contents

  // print everything
  if (this->debug_protocol) {
    ESP_LOGD(TAG, "S21: %s -> %s %s", rcode,
             str_repr(payload, payload_len).c_str(),
             hex_repr(payload, payload_len).c_str());
  }

  // print changed values
  if (this->debug_protocol) {
    auto curr = std::vector<uint8_t>(payload, payload + payload_len);
    if (val_cache[rcode] != curr) {
      const auto prev = val_cache[rcode];
      ESP_LOGI(TAG, "S21 %s changed: %s %s -> %s %s", rcode,
               str_repr(prev.data(), prev.size()).c_str(),
               hex_repr(prev.data(), prev.size()).c_str(),
               str_repr(curr.data(), curr.size()).c_str(),
               hex_repr(curr.data(), curr.size()).c_str());
      val_cache[rcode] = curr;
    }
  }
}

void DaikinS21::handle_nak() {
  ESP_LOGW(TAG, "Rx: NAK from S21 for %s", tx_command);
  if (strcmp(tx_command, *current_query) == 0) {
    ESP_LOGW(TAG, "Removing %s from query pool (assuming unsupported)", tx_command);
    // current_query iterator will be invalidated, recover index and recreate
    const auto index = std::distance(queries.begin(), current_query);
    queries.erase(current_query);
    current_query = queries.begin() + index;
  } else {
    ESP_LOGW(TAG, "Acknowledging %s command despite NAK", tx_command);
    parse_ack();  // don't get stuck retrying unsupported command
  }
}

bool DaikinS21::determine_protocol_version() {
  static constexpr uint8_t G8_version0[4] = {'0',0,0,0};
  static constexpr uint8_t G8_version2or3[4] = {'0','2',0,0};
  static constexpr uint8_t G8_version31plus[4] = {'0','2','0','0'};

  if (std::equal(std::begin(this->G8), std::end(G8), std::begin(G8_version0), std::end(G8_version0))) {
    this->protocol_version = {0,0};
    return true;
  }
  if (std::equal(std::begin(this->G8), std::end(G8), std::begin(G8_version2or3), std::end(G8_version2or3)) && (is_query_active("GY00") == false)) {
    this->protocol_version = {2,0};  // NAK rules out 3.0
    return true;
  }
  switch (this->GY00) {
    case 300:
      if (std::equal(std::begin(this->G8), std::end(G8), std::begin(G8_version2or3), std::end(G8_version2or3))) {
        this->protocol_version = {3,0};  // ACK means 3.0 has support for this query
        return true;
      }
      if (std::equal(std::begin(this->G8), std::end(G8), std::begin(G8_version31plus), std::end(G8_version31plus))) {
        this->protocol_version = {3,1};
        return true;
      }
      break;
    case 320:
      this->protocol_version = {3,2};
      return true;
    case 340:
      this->protocol_version = {3,4};
      return true;
    default:
      break;
  }
  return false; // not detected yet or unsupported
}

void DaikinS21::setup() {
  // populate messages to poll
  // clang-format off
#define S21_EXPERIMENTS 1
  queries = {
      // Protocol version detect:
      "F8", "FY00",
      // Standard:
      "F1", "F5",
      "RG", "RH", "RI", "RL", "RN", "RX",
      "Ra", "Rb", "Rd", "Re",
      // worse:
      // "F9", // better support in RH and Ra
      // redundant:
      // "RB", "RC", "RF",
      // not supported by my units
      // "F6",
      // not decoded yet
      // "F2", "F3", "F4", "F6",
      // "RzB2",
      // "RzC3",
      // "M",
#if S21_EXPERIMENTS
      // Observed BRP device querying these.
      // "RD", "FU0F",
      // Query Experiments
      // "RA", 
      // "RE",
      // "RK", "RM", "RW",
      // "Rg",
#endif
  };
  // clang-format on

  current_query = queries.begin();
}

void DaikinS21::loop() {
  using Result = DaikinSerial::Result;

  switch (serial.service()) {
    case Result::Ack:
      parse_ack();
      break;

    case Result::Idle:
      tx_next();
      break;

    case Result::Nak:
      handle_nak();
      break;

    case Result::Error:
      current_query = queries.end();
      refresh_state = true;
      activate_climate = false;
      activate_swing_mode = false;
      break;

    case Result::Timeout:
      ESP_LOGW(TAG, "Timeout waiting for response to %s", tx_command);
      break;

    default:
    break;
  }
}

void DaikinS21::update() {
  refresh_state = true;

  static bool ready_printed = false;
  if (!ready_printed && this->is_ready()) {
    ESP_LOGI(TAG, "Daikin S21 Ready");
    ready_printed = true;
  }

  if (this->debug_protocol) {
    this->dump_state();
  }
}

void DaikinS21::dump_state() {
  ESP_LOGD(TAG, "** BEGIN STATE *****************************");

  ESP_LOGD(TAG, "  Proto: v%u.%u", this->protocol_version.major, this->protocol_version.minor);
  ESP_LOGD(TAG, "   Mode: %s",
          LOG_STR_ARG(climate::climate_mode_to_string(this->active.mode)));
  ESP_LOGD(TAG, " Action: %s",
          LOG_STR_ARG(climate::climate_action_to_string(this->get_climate_action())));
  ESP_LOGD(TAG, " Target: %.1f C (%.1f F)", c10_c(this->active.setpoint),
          c10_f(this->active.setpoint));
  ESP_LOGD(TAG, "    Fan: %s (%d rpm)",
          daikin_fan_mode_to_string(this->active.fan).c_str(), this->fan_rpm);
  ESP_LOGD(TAG, "  Swing: %s",
          LOG_STR_ARG(climate::climate_swing_mode_to_string(this->active.swing)));
  ESP_LOGD(TAG, " Inside: %.1f C (%.1f F)", c10_c(this->temp_inside),
          c10_f(this->temp_inside));
  ESP_LOGD(TAG, "Outside: %.1f C (%.1f F)", c10_c(this->temp_outside),
          c10_f(this->temp_outside));
  ESP_LOGD(TAG, "   Coil: %.1f C (%.1f F)", c10_c(this->temp_coil),
          c10_f(this->temp_coil));
  ESP_LOGD(TAG, "  Humid: %u%%", this->get_humidity());
  ESP_LOGD(TAG, " Demand: %u", this->get_demand());

  ESP_LOGD(TAG, "** END STATE *****************************");
}

void DaikinS21::set_daikin_climate_settings(climate::ClimateMode mode,
                                            float setpoint,
                                            DaikinFanMode fan_mode) {
  pending.mode = mode;
  pending.setpoint = setpoint;
  pending.fan = fan_mode;
  activate_climate = true;
}

void DaikinS21::set_swing_settings(const climate::ClimateSwingMode swing) {
  pending.swing = swing;
  activate_swing_mode = true;
}

climate::ClimateAction DaikinS21::get_climate_action() {
  switch (get_climate_mode()) {
    // modes where the unit is trying to reach a temperature
    case climate::CLIMATE_MODE_HEAT_COOL:
    case climate::CLIMATE_MODE_COOL:
    case climate::CLIMATE_MODE_HEAT:
    case climate::CLIMATE_MODE_AUTO:
      if (this->demand == 0) {
        return climate::CLIMATE_ACTION_IDLE;
      }
      break;
    // modes where the fan needs to be running for the action to be active
    case climate::CLIMATE_MODE_FAN_ONLY:
      if (this->fan_rpm == 0) {
        return climate::CLIMATE_ACTION_IDLE;
      }
      break;
    case climate::CLIMATE_MODE_DRY:
    default:
      break;
  }
  return this->climate_action;
}

}  // namespace daikin_s21
}  // namespace esphome
