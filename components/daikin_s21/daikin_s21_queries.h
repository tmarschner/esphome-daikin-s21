#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <string_view>
#include <span>
#include "daikin_s21_serial.h"

namespace esphome::daikin_s21 {

class DaikinS21;

using PayloadBuffer = std::array<uint8_t, DaikinSerial::S21_PAYLOAD_SIZE>;

class DaikinQueryValue {
 public:
  DaikinQueryValue() {}
  DaikinQueryValue(const std::string_view command) : command(command) {}

  std::string_view command{};
  PayloadBuffer value{};  // last received value

  // command fetching projection for std::ranges use
  static constexpr std::string_view GetCommand(const DaikinQueryValue &query) { return query.command; }
};

class DaikinQueryState : public DaikinQueryValue {
  using handler_fn = void (DaikinS21::*)(std::span<const uint8_t>);

 public:
  DaikinQueryState() {}
  DaikinQueryState(const std::string_view command, const handler_fn handler = nullptr, const bool is_static = false)
      : DaikinQueryValue(command), handler(handler), is_static(is_static) {}

  handler_fn handler{};
  bool is_static{}; // result never changes
  bool acked{};
  uint8_t naks{};
};

namespace StateQuery {
  inline constexpr std::string_view Basic{"F1"};
  inline constexpr std::string_view OptionalFeatures{"F2"};
  inline constexpr std::string_view OnOffTimer{"F3"};
  inline constexpr std::string_view ErrorStatus{"F4"};
  inline constexpr std::string_view SwingOrHumidity{"F5"};
  inline constexpr std::string_view SpecialModes{"F6"};
  inline constexpr std::string_view DemandAndEcono{"F7"};
  inline constexpr std::string_view OldProtocol{"F8"};
  inline constexpr std::string_view InsideOutsideTemperatures{"F9"};
  // FA
  // FB
  inline constexpr std::string_view ModelCode{"FC"};
  inline constexpr std::string_view IRCounter{"FG"};
  inline constexpr std::string_view V2OptionalFeatures{"FK"};
  // FL
  inline constexpr std::string_view PowerConsumption{"FM"};
  // FN
  // FP
  // FQ
  inline constexpr std::string_view LouvreAngle{"FR"};
  // FS
  // FT
  inline constexpr std::string_view V3OptionalFeatures{"FU00"};
  inline constexpr std::string_view AllowedTemperatureRange{"FU02"};
  // FU04
  inline constexpr std::string_view ModelName{"FU05"};
  inline constexpr std::string_view ProductionInformation{"FU15"};
  inline constexpr std::string_view ProductionOrder{"FU25"};
  inline constexpr std::string_view IndoorProductionInformation{"FU35"};
  inline constexpr std::string_view OutdoorProductionInformation{"FU45"};
  // FV
  // FX00
  // FX10
  // FX20
  // FX30
  // FX40
  // FX50
  // FX60
  // FX70
  // FX80
  // FX90
  // FXA0
  // FXB0
  // FXC0
  // FXD0
  // FXE0
  // FXF0
  // FX01
  // FX11
  // FX21
  // FX31
  // FX41
  // FX51
  // FX61
  // FX71
  // FX81
  inline constexpr std::string_view NewProtocol{"FY00"};
  // FY10
  // FY20
} // namespace StateQuery

namespace EnvironmentQuery {
  inline constexpr std::string_view PowerOnOff{"RA"};
  inline constexpr std::string_view IndoorUnitMode{"RB"};
  inline constexpr std::string_view TemperatureSetPoint{"RC"};
  inline constexpr std::string_view OnTimerSetting{"RD"};
  inline constexpr std::string_view OffTimerSetting{"RE"};
  inline constexpr std::string_view SwingMode{"RF"};
  inline constexpr std::string_view FanMode{"RG"};
  inline constexpr std::string_view InsideTemperature{"RH"};
  inline constexpr std::string_view LiquidTemperature{"RI"};
  inline constexpr std::string_view FanSetPoint{"RK"};
  inline constexpr std::string_view FanSpeed{"RL"};
  inline constexpr std::string_view LouvreAngleSetPoint{"RM"};
  inline constexpr std::string_view VerticalSwingAngle{"RN"};
  // RW
  inline constexpr std::string_view TargetTemperature{"RX"}; // (not set point, see details)
  inline constexpr std::string_view OutsideTemperature{"Ra"};
  inline constexpr std::string_view IndoorFrequencyCommandSignal{"Rb"};
  inline constexpr std::string_view CompressorFrequency{"Rd"};
  inline constexpr std::string_view IndoorHumidity{"Re"};
  inline constexpr std::string_view CompressorOnOff{"Rg"};
  inline constexpr std::string_view UnitState{"RzB2"};
  inline constexpr std::string_view SystemState{"RzC3"};
  // Rz52
  // Rz72
} // namespace EnvironmentQuery

namespace MiscQuery {
  inline constexpr std::string_view Model{"M"};
  inline constexpr std::string_view Version{"V"};
} // namespace MiscQuery

namespace StateCommand {
  inline constexpr std::string_view PowerModeTempFan{"D1"};
  // D2
  inline constexpr std::string_view OnOffTimer{"D3"};
  inline constexpr std::string_view LouvreSwing{"D5"};
  inline constexpr std::string_view Powerful{"D6"};
  inline constexpr std::string_view Econo{"D7"};
  // DH
  // DJ
  // DR
}

} // namespace esphome::daikin_s21