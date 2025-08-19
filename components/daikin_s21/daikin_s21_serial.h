#pragma once

#include <cstdint>
#include <span>
#include <string_view>
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"

namespace esphome::daikin_s21 {

class DaikinS21;

class DaikinSerial : public Component,
                     public Parented<DaikinS21> {
 public:
  static constexpr uint32_t S21_MAX_COMMAND_SIZE{4};
  static constexpr uint32_t S21_PAYLOAD_SIZE{4};

  enum class Result : uint8_t {
    Ack,
    Nak,
    Timeout,
    Error,
  };

  DaikinSerial(uart::UARTComponent *uart) : uart(*uart) {}

  void setup() override;
  void loop() override;

  void set_debug(bool set) { this->debug = set; }

  void send_frame(std::string_view cmd, std::span<const uint8_t> payload = {});

protected:
  static constexpr const char * timer_name = "timer";
  static constexpr uint32_t ack_delay_period_ms{45};      // official remote delay time before ACKing a response
  static constexpr uint32_t next_tx_delay_period_ms{35};  // official remote delay time between commands
  static constexpr uint32_t error_delay_period_ms{3000};  // cooldown time when something goes wrong
  static constexpr uint32_t rx_timout_period_ms{250};   // waiting for a response from the unit

  enum class CommState : uint8_t {
    CommandAck,
    QueryAck,
    QueryStx,
    QueryEtx,
  };

  void set_ack_timeout(int bytes_received);
  void ack_timeout_handler();
  void set_busy_timeout(uint32_t delay_ms);
  void busy_timeout_handler();
  void set_rx_timeout();
  void rx_timeout_handler();

  uart::UARTComponent &uart;
  bool debug{};
  CommState comm_state{};
  std::vector<uint8_t> response{};
};

} // namespace esphome::daikin_s21
