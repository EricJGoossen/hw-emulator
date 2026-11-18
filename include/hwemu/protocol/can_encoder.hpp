#pragma once

// Absolute encoders on CAN, as taproot's CanEncoder and aruwsrc's
// LampreyEncoder read them (ids 0x1E0..0x1E7, little-endian):
//
//   CanEncoder:     [0..1] angle, 0..4095 per turn   [2..3] field strength (gauss)
//   LampreyEncoder: [0..1] angle in centidegrees, 0..35999

#include <cstdint>
#include <optional>

#include "hwemu/protocol/can_frame.hpp"

namespace hwemu::protocol::can_encoder {

inline constexpr std::uint32_t kFirstId = 0x1E0;
inline constexpr std::uint32_t kLastId = 0x1E7;
inline constexpr std::uint16_t kCounts = 4096;

struct Reading {
  std::uint16_t counts = 0;
  std::uint16_t gauss = 0;
  friend bool operator==(const Reading&, const Reading&) = default;
};

CanFrame Encode(std::uint32_t id, const Reading& reading);
std::optional<Reading> Decode(const CanFrame& frame);

CanFrame EncodeLamprey(std::uint32_t id, std::uint16_t centidegrees);
std::optional<std::uint16_t> DecodeLamprey(const CanFrame& frame);

// Radians (any value) to the wire's units, wrapped into one turn.
std::uint16_t AngleToCounts(double radians, std::uint16_t counts_per_turn = kCounts);
std::uint16_t AngleToCentidegrees(double radians);

}  // namespace hwemu::protocol::can_encoder
