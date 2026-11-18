// The CAN codecs: DJI motors, CAN encoders, the turret MCB, T-Motor.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <numbers>

#include "hwemu/protocol/bytes.hpp"
#include "hwemu/protocol/can_encoder.hpp"
#include "hwemu/protocol/can_frame.hpp"
#include "hwemu/protocol/dji_motor_can.hpp"
#include "hwemu/protocol/tmotor_can.hpp"
#include "hwemu/protocol/turret_mcb_can.hpp"

namespace hwemu::protocol {

std::string ToString(const CanFrame& frame) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), frame.extended ? "%08X [%u]" : "%03X [%u]", frame.id, frame.len);
  std::string out = buf;
  for (std::uint8_t i = 0; i < frame.len && i < 8; ++i) {
    std::snprintf(buf, sizeof(buf), " %02X", frame.data[i]);
    out += buf;
  }
  return out;
}

namespace {

template <typename T>
T ClampRound(double v) {
  const double lo = static_cast<double>(std::numeric_limits<T>::min());
  const double hi = static_cast<double>(std::numeric_limits<T>::max());
  return static_cast<T>(std::lround(std::clamp(v, lo, hi)));
}

double WrapTwoPi(double radians) {
  constexpr double kTwoPi = 2.0 * std::numbers::pi;
  double w = std::fmod(radians, kTwoPi);
  if (w < 0.0) {
    w += kTwoPi;
  }
  return w;
}

double WrapPi(double radians) {
  constexpr double kPi = std::numbers::pi;
  double w = WrapTwoPi(radians + kPi) - kPi;
  if (w <= -kPi) {
    w += 2.0 * kPi;
  }
  return w;
}

CanFrame Standard(std::uint32_t id, std::uint8_t len) {
  CanFrame f;
  f.id = id;
  f.len = len;
  return f;
}

}  // namespace

// ---- DJI motors -----------------------------------------------------------
namespace dji_motor {

bool IsFeedbackId(std::uint32_t id) { return id >= kFirstFeedbackId && id <= kLastFeedbackId; }

std::optional<CommandSlot> CommandSlotFor(std::uint32_t feedback_id, bool current_frame) {
  if (!IsFeedbackId(feedback_id)) {
    return std::nullopt;
  }
  const int n = static_cast<int>(feedback_id - kFirstFeedbackId);  // 0..10
  if (n < 4) {
    return CommandSlot{kCmdLow, n};
  }
  if (n < 8) {
    return CommandSlot{current_frame ? kCmdHighCurrent : kCmdHigh, n - 4};
  }
  return CommandSlot{current_frame ? kCmdExtCurrent : kCmdExt, n - 8};
}

CanFrame EncodeFeedback(std::uint32_t feedback_id, const Feedback& feedback) {
  CanFrame f = Standard(feedback_id, 8);
  PutU16Be(&f.data[0], feedback.encoder);
  PutI16Be(&f.data[2], feedback.rpm);
  PutI16Be(&f.data[4], feedback.current);
  f.data[6] = static_cast<std::uint8_t>(feedback.temperature);
  f.data[7] = 0;
  return f;
}

std::optional<Feedback> DecodeFeedback(const CanFrame& frame) {
  if (frame.extended || !IsFeedbackId(frame.id) || frame.len < 7) {
    return std::nullopt;
  }
  Feedback fb;
  fb.encoder = GetU16Be(&frame.data[0]);
  fb.rpm = GetI16Be(&frame.data[2]);
  fb.current = GetI16Be(&frame.data[4]);
  fb.temperature = static_cast<std::int8_t>(frame.data[6]);
  return fb;
}

CanFrame EncodeCommand(std::uint32_t frame_id, const std::array<std::int16_t, 4>& slots) {
  CanFrame f = Standard(frame_id, 8);
  for (int i = 0; i < 4; ++i) {
    PutI16Be(&f.data[static_cast<std::size_t>(2 * i)], slots[static_cast<std::size_t>(i)]);
  }
  return f;
}

std::optional<std::int16_t> DecodeCommand(const CanFrame& frame, const CommandSlot& slot) {
  if (frame.extended || frame.id != slot.frame_id) {
    return std::nullopt;
  }
  const auto offset = static_cast<std::size_t>(2 * slot.slot);
  if (offset + 2 > frame.len) {
    return std::nullopt;
  }
  return GetI16Be(&frame.data[offset]);
}

}  // namespace dji_motor

// ---- CAN encoders ---------------------------------------------------------
namespace can_encoder {

CanFrame Encode(std::uint32_t id, const Reading& reading) {
  CanFrame f = Standard(id, 8);
  PutU16Le(&f.data[0], reading.counts);
  PutU16Le(&f.data[2], reading.gauss);
  return f;
}

std::optional<Reading> Decode(const CanFrame& frame) {
  if (frame.extended || frame.len < 4) {
    return std::nullopt;
  }
  return Reading{GetU16Le(&frame.data[0]), GetU16Le(&frame.data[2])};
}

CanFrame EncodeLamprey(std::uint32_t id, std::uint16_t centidegrees) {
  CanFrame f = Standard(id, 8);
  PutU16Le(&f.data[0], centidegrees);
  return f;
}

std::optional<std::uint16_t> DecodeLamprey(const CanFrame& frame) {
  if (frame.extended || frame.len < 2) {
    return std::nullopt;
  }
  return GetU16Le(&frame.data[0]);
}

std::uint16_t AngleToCounts(double radians, std::uint16_t counts_per_turn) {
  const double turns = WrapTwoPi(radians) / (2.0 * std::numbers::pi);
  auto counts = static_cast<std::uint32_t>(std::floor(turns * counts_per_turn));
  return static_cast<std::uint16_t>(counts % counts_per_turn);
}

std::uint16_t AngleToCentidegrees(double radians) {
  const double degrees = WrapTwoPi(radians) * 180.0 / std::numbers::pi;
  return static_cast<std::uint16_t>(static_cast<std::uint32_t>(std::floor(degrees * 100.0)) % 36000U);
}

}  // namespace can_encoder

// ---- Turret MCB -----------------------------------------------------------
namespace turret_mcb {

std::int16_t AngleToFixed(double radians) {
  return ClampRound<std::int16_t>(WrapPi(radians) / kAngleRadPerLsb);
}
double FixedToAngle(std::int16_t fixed) { return fixed * kAngleRadPerLsb; }
std::int16_t RateToRaw(double rad_per_s) {
  return ClampRound<std::int16_t>(rad_per_s * kGyroLsbPerRadPerS);
}
double RawToRate(std::int16_t raw) { return raw / kGyroLsbPerRadPerS; }
std::int16_t AccelerationToRaw(double m_per_s2) { return ClampRound<std::int16_t>(m_per_s2 * 100.0); }
double RawToAcceleration(std::int16_t raw) { return raw * 0.01; }

CanFrame EncodeAxis(std::uint32_t id, const AxisMessage& m) {
  CanFrame f = Standard(id, 7);
  PutI16Le(&f.data[0], m.angle);
  PutI16Le(&f.data[2], m.rate);
  PutI16Le(&f.data[4], m.acceleration);
  f.data[6] = m.seq;
  return f;
}

std::optional<AxisMessage> DecodeAxis(const CanFrame& frame) {
  if (frame.extended || frame.len < 7 || frame.id < kXAxisId || frame.id > kZAxisId) {
    return std::nullopt;
  }
  return AxisMessage{GetI16Le(&frame.data[0]), GetI16Le(&frame.data[2]), GetI16Le(&frame.data[4]), frame.data[6]};
}

CanFrame EncodeStatus(const StatusMessage& m) {
  CanFrame f = Standard(kStatusId, 4);
  f.data[0] = m.bitmask;
  f.data[1] = static_cast<std::uint8_t>(m.imu_state);
  PutI16Le(&f.data[2], m.temperature_centi_c);
  return f;
}

std::optional<StatusMessage> DecodeStatus(const CanFrame& frame) {
  if (frame.extended || frame.id != kStatusId || frame.len < 4) {
    return std::nullopt;
  }
  return StatusMessage{frame.data[0], static_cast<ImuState>(frame.data[1]), GetI16Le(&frame.data[2])};
}

CanFrame EncodeCommand(std::uint8_t bitmask) {
  CanFrame f = Standard(kCommandId, 1);
  f.data[0] = bitmask;
  return f;
}

std::optional<std::uint8_t> DecodeCommand(const CanFrame& frame) {
  if (frame.extended || frame.id != kCommandId || frame.len < 1) {
    return std::nullopt;
  }
  return frame.data[0];
}

CanFrame EncodeRequest(std::uint32_t request_id) { return Standard(request_id, 0); }

CanFrame EncodeSyncResponse(std::uint32_t micros) {
  CanFrame f = Standard(kSyncResponseId, 4);
  PutU32Le(&f.data[0], micros);
  return f;
}

std::optional<std::uint32_t> DecodeSyncResponse(const CanFrame& frame) {
  if (frame.extended || frame.id != kSyncResponseId || frame.len < 4) {
    return std::nullopt;
  }
  return GetU32Le(&frame.data[0]);
}

CanFrame EncodeCalibrationSamples(std::uint16_t samples) {
  CanFrame f = Standard(kCalibrationSamplesId, 2);
  PutU16Le(&f.data[0], samples);
  return f;
}

std::optional<std::uint16_t> DecodeCalibrationSamples(const CanFrame& frame) {
  if (frame.extended || frame.id != kCalibrationSamplesId || frame.len < 2) {
    return std::nullopt;
  }
  return GetU16Le(&frame.data[0]);
}

std::array<double, 3> MountingMessage::Values() const {
  std::array<double, 3> out{};
  for (std::size_t i = 0; i < 3; ++i) {
    if (part == TransformPart::kTranslation) {
      out[i] = static_cast<std::int16_t>(raw[i]) / (32767.0 / 2.0);
    } else {
      out[i] = raw[i] / (65535.0 / (2.0 * std::numbers::pi));
    }
  }
  return out;
}

CanFrame EncodeMounting(const MountingMessage& m) {
  CanFrame f = Standard(kImuMountingId, 8);
  f.data[0] = static_cast<std::uint8_t>(m.imu);
  f.data[1] = static_cast<std::uint8_t>(m.part);
  for (std::size_t i = 0; i < 3; ++i) {
    PutU16Le(&f.data[2 + (2 * i)], m.raw[i]);
  }
  return f;
}

std::optional<MountingMessage> DecodeMounting(const CanFrame& frame) {
  if (frame.extended || frame.id != kImuMountingId || frame.len < 8) {
    return std::nullopt;
  }
  MountingMessage m;
  m.imu = static_cast<RemoteImuType>(frame.data[0]);
  m.part = static_cast<TransformPart>(frame.data[1]);
  for (std::size_t i = 0; i < 3; ++i) {
    m.raw[i] = GetU16Le(&frame.data[2 + (2 * i)]);
  }
  return m;
}

}  // namespace turret_mcb

// ---- T-Motor --------------------------------------------------------------
namespace tmotor {

CanFrame EncodeFeedback(std::uint8_t motor_id, const Feedback& fb) {
  CanFrame f;
  f.id = FeedbackId(motor_id);
  f.extended = true;
  f.len = 8;
  PutI16Be(&f.data[0], fb.position);
  PutI16Be(&f.data[2], fb.speed);
  PutI16Be(&f.data[4], fb.current);
  f.data[6] = static_cast<std::uint8_t>(fb.temperature);
  f.data[7] = fb.fault;
  return f;
}

std::optional<Feedback> DecodeFeedback(const CanFrame& frame) {
  if (!frame.extended || (frame.id & 0xFFFFFF00U) != kFeedbackBase || frame.len < 8) {
    return std::nullopt;
  }
  Feedback fb;
  fb.position = GetI16Be(&frame.data[0]);
  fb.speed = GetI16Be(&frame.data[2]);
  fb.current = GetI16Be(&frame.data[4]);
  fb.temperature = static_cast<std::int8_t>(frame.data[6]);
  fb.fault = frame.data[7];
  return fb;
}

CanFrame EncodeCurrentCommand(std::uint8_t motor_id, std::int32_t milliamps) {
  CanFrame f;
  f.id = CommandId(motor_id, kCurrentMode);
  f.extended = true;
  f.len = 8;
  PutI32Be(&f.data[0], milliamps);
  return f;
}

std::optional<std::int32_t> DecodeCurrentCommand(const CanFrame& frame, std::uint8_t motor_id) {
  if (!frame.extended || frame.id != CommandId(motor_id, kCurrentMode) || frame.len < 4) {
    return std::nullopt;
  }
  return GetI32Be(&frame.data[0]);
}

}  // namespace tmotor

}  // namespace hwemu::protocol
