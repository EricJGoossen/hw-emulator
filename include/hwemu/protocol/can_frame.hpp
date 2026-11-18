#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace hwemu::protocol {

// One classic CAN frame, as modm::can::Message carries it.
struct CanFrame {
  std::uint32_t id = 0;
  bool extended = false;
  bool rtr = false;
  std::uint8_t len = 8;
  std::array<std::uint8_t, 8> data{};

  friend bool operator==(const CanFrame&, const CanFrame&) = default;
};

// "0x201 [8] 12 34 ..." -- candump style, for logs and test failures.
std::string ToString(const CanFrame& frame);

}  // namespace hwemu::protocol
