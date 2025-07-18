#include "daikin_s21_binary_sensor.h"

namespace esphome {
namespace daikin_s21 {

static const char *const TAG = "daikin_s21.binary_sensor";

void DaikinS21BinarySensor::update() {
  if (!this->s21->is_ready())
    return;

  const auto unit_state = s21->unit_state;
  const auto system_state = s21->system_state;
  if (this->powerful_sensor_ != nullptr) {
    this->powerful_sensor_->publish_state(unit_state & 0x1);
  }
  if (this->defrost_sensor_ != nullptr) {
    this->defrost_sensor_->publish_state(unit_state & 0x2);
  }
  if (this->active_sensor_ != nullptr) {
    this->active_sensor_->publish_state(unit_state & 0x4);  // unit
  }
  if (this->valve_sensor_ != nullptr) {
    this->valve_sensor_->publish_state(system_state & 0x04);  // system
  }
  if (this->short_cycle_sensor_ != nullptr) {
    this->short_cycle_sensor_->publish_state((system_state & 0x01) == 0x00);  // invert for ESPHome logic
  }
  if (this->system_defrost_sensor_ != nullptr) {
    this->system_defrost_sensor_->publish_state(system_state & 0x08);
  }
}

void DaikinS21BinarySensor::dump_config() {
  ESP_LOGCONFIG(TAG, "Daikin S21 Binary Sensor:");
  LOG_BINARY_SENSOR("  ", "Powerful", this->powerful_sensor_);
  LOG_BINARY_SENSOR("  ", "Defrost", this->defrost_sensor_);
  LOG_BINARY_SENSOR("  ", "Active", this->active_sensor_);
  LOG_BINARY_SENSOR("  ", "Valve", this->valve_sensor_);
  LOG_BINARY_SENSOR("  ", "Short Cycle", this->short_cycle_sensor_);
  LOG_BINARY_SENSOR("  ", "System Defrost", this->system_defrost_sensor_);
}

}  // namespace daikin_s21
}  // namespace esphome
