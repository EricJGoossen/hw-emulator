#pragma once

// T-Motor AK80-9 in servo mode, as aruwsrc's Tmotor_AK809 speaks to it
// (extended ids, big-endian):
//
// Commands, to the motor:
//   (0x01 << 8) | id  current mode: int32 current, mA
//   (0x05 << 8) | id  origin: data[0] 1 sets the origin, 2 asks for it
// Feedback, from the motor, at 0x2900 | id:
//   [0..1] position, int16, 0.1 deg of the output shaft
//   [2..3] speed, int16, rotor rpm (taproot divides by the 9:1 gearbox)
//   [4..5] current, int16, 0.01 A
//   [6]    temperature, int8 degC
//   [7]    fault code

#include <cstdint>
#include <optional>

#include "hwemu/protocol/can_frame.hpp"

namespace hwemu::protocol::tmotor {

inline constexpr std::uint32_t kFeedbackBase = 0x2900;
inline constexpr std::uint32_t kCurrentMode = 0x01;
inline constexpr std::uint32_t kOriginMode = 0x05;
inline constexpr std::int32_t kMaxCurrentMilliamps = 60000;

struct Feedback {
  std::int16_t position = 0;
  std::int16_t speed = 0;
  std::int16_t current = 0;
  std::int8_t temperature = 0;
  std::uint8_t fault = 0;
  friend bool operator==(const Feedback&, const Feedback&) = default;
};

inline std::uint32_t FeedbackId(std::uint8_t motor_id) { return kFeedbackBase | motor_id; }
inline std::uint32_t CommandId(std::uint8_t motor_id, std::uint32_t mode) {
  return (mode << 8U) | motor_id;
}

CanFrame EncodeFeedback(std::uint8_t motor_id, const Feedback& feedback);
std::optional<Feedback> DecodeFeedback(const CanFrame& frame);
CanFrame EncodeCurrentCommand(std::uint8_t motor_id, std::int32_t milliamps);
// The current a frame commands, if it's motor_id's current-mode frame.
std::optional<std::int32_t> DecodeCurrentCommand(const CanFrame& frame, std::uint8_t motor_id);

}  // namespace hwemu::protocol::tmotor
