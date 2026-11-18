#include "hwemu/protocol/crc.hpp"

#include <array>

namespace hwemu::protocol {
namespace {

// Tables built once from the polynomials, rather than pasted in.
constexpr std::array<std::uint8_t, 256> ReflectedTable8(std::uint8_t poly) {
  std::array<std::uint8_t, 256> table{};
  for (unsigned i = 0; i < 256; ++i) {
    auto crc = static_cast<std::uint8_t>(i);
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) != 0 ? static_cast<std::uint8_t>((crc >> 1U) ^ poly)
                            : static_cast<std::uint8_t>(crc >> 1U);
    }
    table[i] = crc;
  }
  return table;
}

constexpr std::array<std::uint16_t, 256> ReflectedTable16(std::uint16_t poly) {
  std::array<std::uint16_t, 256> table{};
  for (unsigned i = 0; i < 256; ++i) {
    auto crc = static_cast<std::uint16_t>(i);
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) != 0 ? static_cast<std::uint16_t>((crc >> 1U) ^ poly)
                            : static_cast<std::uint16_t>(crc >> 1U);
    }
    table[i] = crc;
  }
  return table;
}

constexpr std::array<std::uint8_t, 256> MsbFirstTable8(std::uint8_t poly) {
  std::array<std::uint8_t, 256> table{};
  for (unsigned i = 0; i < 256; ++i) {
    auto crc = static_cast<std::uint8_t>(i);
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80U) != 0 ? static_cast<std::uint8_t>((crc << 1U) ^ poly)
                               : static_cast<std::uint8_t>(crc << 1U);
    }
    table[i] = crc;
  }
  return table;
}

constexpr auto kCrc8Dji = ReflectedTable8(0x8C);
constexpr auto kCrc16Dji = ReflectedTable16(0x8408);
constexpr auto kCrc8Ld19 = MsbFirstTable8(0x4D);

}  // namespace

std::uint8_t Crc8Dji(std::span<const std::uint8_t> data, std::uint8_t init) {
  std::uint8_t crc = init;
  for (std::uint8_t byte : data) {
    crc = kCrc8Dji[crc ^ byte];
  }
  return crc;
}

std::uint16_t Crc16Dji(std::span<const std::uint8_t> data, std::uint16_t init) {
  std::uint16_t crc = init;
  for (std::uint8_t byte : data) {
    crc = static_cast<std::uint16_t>((crc >> 8U) ^ kCrc16Dji[(crc ^ byte) & 0xFFU]);
  }
  return crc;
}

std::uint8_t Crc8Ld19(std::span<const std::uint8_t> data, std::uint8_t init) {
  std::uint8_t crc = init;
  for (std::uint8_t byte : data) {
    crc = kCrc8Ld19[crc ^ byte];
  }
  return crc;
}

}  // namespace hwemu::protocol
