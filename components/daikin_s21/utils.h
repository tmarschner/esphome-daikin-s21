#pragma once

#include <cstdint>
#include <string>
#include <span>

namespace esphome::daikin_s21 {

// printf format specifier macros for std::string_view
#define PRI_SV ".*s"
#define PRI_SV_ARGS(x) (x).size(), (x).data()

std::string hex_repr(std::span<const uint8_t> bytes);
std::string str_repr(std::span<const uint8_t> bytes);

static constexpr uint8_t ahex_digit(uint8_t digit) { return (digit >= 'A') ? (digit - 'A') + 10 : digit - '0'; }
static constexpr uint8_t ahex_u8_le(uint8_t first, uint8_t second) { return (ahex_digit(second) << 4) | ahex_digit(first); }

} // namespace esphome::daikin_s21
