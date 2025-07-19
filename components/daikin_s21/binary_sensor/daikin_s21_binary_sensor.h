#pragma once

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "../s21.h"

namespace esphome {
namespace daikin_s21 {

class DaikinS21BinarySensor : public PollingComponent, public DaikinS21Client {
 public:
  void update() override;
  void dump_config() override;

  void set_powerful_sensor(binary_sensor::BinarySensor *sensor) {
    this->powerful_sensor_ = sensor;
  }
  void set_defrost_sensor(binary_sensor::BinarySensor *sensor) {
    this->defrost_sensor_ = sensor;
  }
  void set_active_sensor(binary_sensor::BinarySensor *sensor) {
    this->active_sensor_ = sensor;
  }
  void set_valve_sensor(binary_sensor::BinarySensor *sensor) {
    this->valve_sensor_ = sensor;
  }
  void set_short_cycle_sensor(binary_sensor::BinarySensor *sensor) {
    this->short_cycle_sensor_ = sensor;
  }
  void set_system_defrost_sensor(binary_sensor::BinarySensor *sensor) {
    this->system_defrost_sensor_ = sensor;
  }

 protected:
  binary_sensor::BinarySensor *powerful_sensor_{nullptr};
  binary_sensor::BinarySensor *defrost_sensor_{nullptr};
  binary_sensor::BinarySensor *active_sensor_{nullptr};
  binary_sensor::BinarySensor *valve_sensor_{nullptr};
  binary_sensor::BinarySensor *short_cycle_sensor_{nullptr};
  binary_sensor::BinarySensor *system_defrost_sensor_{nullptr};
};

}  // namespace daikin_s21
}  // namespace esphome
