#include "daikin_s21_queries.h"
#include "daikin_s21_serial.h"
#include "s21.h"
#include "utils.h"

namespace esphome::daikin_s21 {

static const char *const TAG = "daikin_s21.queries";

/**
 * Copy the last result into the query instance and records the length.
 *
 * Redirects software version to the static buffer.
 */
void DaikinQueryValue::set_value(const std::span<const uint8_t> payload) {
  const auto buffer = this->command == MiscQuery::SoftwareVersion ? std::span<uint8_t>(DaikinQueryValue::software_version) : std::span<uint8_t>(this->value_buffer);
  if (payload.size() <= buffer.size()) {
    std::ranges::copy(payload, buffer.begin());
    this->value_length = payload.size();
  } else {
    ESP_LOGI(TAG, "result too long to cache %" PRI_SV ": %s", PRI_SV_ARGS(this->command), hex_repr(payload).c_str());
  }
}

} // namespace esphome::daikin_s21
