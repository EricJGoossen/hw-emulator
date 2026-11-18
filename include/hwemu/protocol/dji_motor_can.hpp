#pragma once

// DJI motor controllers on CAN (C620/M3508, C610/M2006, GM6020, GM3510,
// 820R), as taproot's DjiMotor and DjiMotorTxHandler speak to them.
//
// Feedback, from the controller at 0x200 + n (n = 1..11), big-endian:
//   [0..1] rotor encoder, 0..8191 per rotor turn
//   [2..3] rotor speed, rpm (int16)
//   [4..5] torque current, raw (int16; scale per controller)
//   [6]    temperature, degC (int8)
//
// Commands, to the controllers, four big-endian int16 slots per frame:
//   0x200 -- feedback ids 0x201..0x204
//   0x1FF -- 0x205..0x208 (GM6020 1..4 voltage, or C620/C610 5..8)
//   0x1FE -- 0x205..0x208 as GM6020 current control
//   0x2FF -- 0x209..0x20B (GM6020 5..7 voltage)
//   0x2FE -- 0x209..0x20B as GM6020 current control

#include <array>
#include <cstdint>
#include <optional>

#include "hwemu/protocol/can_frame.hpp"

namespace hwemu::protocol::dji_motor {

inline constexpr std::uint32_t kCmdLow = 0x200;
inline constexpr std::uint32_t kCmdHigh = 0x1FF;
inline constexpr std::uint32_t kCmdHighCurrent = 0x1FE;
inline constexpr std::uint32_t kCmdExt = 0x2FF;
inline constexpr std::uint32_t kCmdExtCurrent = 0x2FE;
inline constexpr std::uint32_t kFeedbackBase = 0x200;
inline constexpr std::uint32_t kFirstFeedbackId = 0x201;
inline constexpr std::uint32_t kLastFeedbackId = 0x20B;
inline constexpr std::uint16_t kEncoderCounts = 8192;

struct Feedback {
  std::uint16_t encoder = 0;
  std::int16_t rpm = 0;
  std::int16_t current = 0;
  std::int8_t temperature = 0;
  friend bool operator==(const Feedback&, const Feedback&) = default;
};

// Where a controller's command lives: which frame, and which of its 4 slots.
struct CommandSlot {
  std::uint32_t frame_id = 0;
  int slot = 0;
  friend bool operator==(const CommandSlot&, const CommandSlot&) = default;
};

bool IsFeedbackId(std::uint32_t id);
// For feedback id 0x201..0x20B. `current_frame` picks the GM6020 current
// control frames (0x1FE / 0x2FE) for ids above 0x204; ignored below.
std::optional<CommandSlot> CommandSlotFor(std::uint32_t feedback_id, bool current_frame);

CanFrame EncodeFeedback(std::uint32_t feedback_id, const Feedback& feedback);
std::optional<Feedback> DecodeFeedback(const CanFrame& frame);

CanFrame EncodeCommand(std::uint32_t frame_id, const std::array<std::int16_t, 4>& slots);
std::optional<std::int16_t> DecodeCommand(const CanFrame& frame, const CommandSlot& slot);

}  // namespace hwemu::protocol::dji_motor
