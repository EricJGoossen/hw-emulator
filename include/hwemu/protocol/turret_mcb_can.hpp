#pragma once

// The turret MCB's CAN protocol, as aruwsrc's TurretMCBCanComm speaks it
// (all little-endian, standard ids):
//
// Turret MCB -> chassis MCB
//   0x1FB/0x1FC/0x1FD  X/Y/Z axis: int16 angle (2*pi/65535 rad per LSB),
//                      int16 rate (MPU6500 counts, 16.384 * 180/pi per rad/s),
//                      int16 acceleration (cm/s^2), u8 sequence -- 7 bytes.
//                      X is roll, Y pitch, Z yaw; one sequence number ties a
//                      set of three together, X first.
//   0x1FA  status: u8 bitmask (bit0 kicker limit switch, bit1 agitator),
//          u8 IMU state, int16 temperature (centi-degC)
//   0x1F8  time-sync request            -> chassis answers 0x1F9 u32 micros
//   0x1F3  calibration-samples request  -> chassis answers 0x1F4 u16 samples
//   0x1F5  IMU-mounting request         -> chassis answers 0x1F6 (below)
//
// Chassis MCB -> turret MCB
//   0x1F7  command: u8 bitmask (bit0 open hopper, bit1 recalibrate IMU,
//          bit2 laser on)
//   0x1F6  mounting transform part: u8 imu type (0 BMI088, 1 ISM330,
//          2 MPU6500), u8 part (0 translation, 1 rotation), 3 components --
//          translation int16 at 32767/2 per m, rotation uint16 roll/pitch/yaw
//          in [0, 2pi) at 65535/(2pi) per rad.

#include <array>
#include <cstdint>
#include <optional>

#include "hwemu/protocol/can_frame.hpp"

namespace hwemu::protocol::turret_mcb {

inline constexpr std::uint32_t kCalibrationSamplesRequestId = 0x1F3;
inline constexpr std::uint32_t kCalibrationSamplesId = 0x1F4;
inline constexpr std::uint32_t kImuMountingRequestId = 0x1F5;
inline constexpr std::uint32_t kImuMountingId = 0x1F6;
inline constexpr std::uint32_t kCommandId = 0x1F7;
inline constexpr std::uint32_t kSyncRequestId = 0x1F8;
inline constexpr std::uint32_t kSyncResponseId = 0x1F9;
inline constexpr std::uint32_t kStatusId = 0x1FA;
inline constexpr std::uint32_t kXAxisId = 0x1FB;
inline constexpr std::uint32_t kYAxisId = 0x1FC;
inline constexpr std::uint32_t kZAxisId = 0x1FD;

// MPU6500's +/-2000 dps scale as taproot writes it: 16.384 LSB per deg/s.
inline constexpr double kGyroLsbPerRadPerS = 16.384 * 180.0 / 3.14159265358979323846;
inline constexpr double kAngleRadPerLsb = 2.0 * 3.14159265358979323846 / 65535.0;

enum class ImuState : std::uint8_t {
  kNotConnected = 0,
  kNotCalibrated = 1,
  kCalibrating = 2,
  kCalibrated = 3,
};

enum CommandBits : std::uint8_t {
  kOpenHopper = 1U << 0U,
  kRecalibrateImu = 1U << 1U,
  kLaserOn = 1U << 2U,
};

enum StatusBits : std::uint8_t {
  kKickerLimitSwitch = 1U << 0U,
  kAgitatorLimitSwitch = 1U << 1U,
};

struct AxisMessage {
  std::int16_t angle = 0;
  std::int16_t rate = 0;
  std::int16_t acceleration = 0;
  std::uint8_t seq = 0;
  friend bool operator==(const AxisMessage&, const AxisMessage&) = default;
};

struct StatusMessage {
  std::uint8_t bitmask = 0;
  ImuState imu_state = ImuState::kNotConnected;
  std::int16_t temperature_centi_c = 0;
  friend bool operator==(const StatusMessage&, const StatusMessage&) = default;
};

enum class RemoteImuType : std::uint8_t { kBmi088 = 0, kIsm330 = 1, kMpu6500 = 2 };
enum class TransformPart : std::uint8_t { kTranslation = 0, kRotation = 1 };

struct MountingMessage {
  RemoteImuType imu = RemoteImuType::kBmi088;
  TransformPart part = TransformPart::kTranslation;
  std::array<std::uint16_t, 3> raw{};  // int16 for translation, uint16 rotation
  // In meters, or radians in [0, 2pi).
  std::array<double, 3> Values() const;
};

// Unit conversions, each clamped to the field's range.
std::int16_t AngleToFixed(double radians);  // wrapped to (-pi, pi] first
double FixedToAngle(std::int16_t fixed);
std::int16_t RateToRaw(double rad_per_s);
double RawToRate(std::int16_t raw);
std::int16_t AccelerationToRaw(double m_per_s2);
double RawToAcceleration(std::int16_t raw);

CanFrame EncodeAxis(std::uint32_t id, const AxisMessage& message);
std::optional<AxisMessage> DecodeAxis(const CanFrame& frame);
CanFrame EncodeStatus(const StatusMessage& message);
std::optional<StatusMessage> DecodeStatus(const CanFrame& frame);

CanFrame EncodeCommand(std::uint8_t bitmask);
std::optional<std::uint8_t> DecodeCommand(const CanFrame& frame);
CanFrame EncodeRequest(std::uint32_t request_id);  // 0x1F3, 0x1F5 or 0x1F8
CanFrame EncodeSyncResponse(std::uint32_t micros);
std::optional<std::uint32_t> DecodeSyncResponse(const CanFrame& frame);
CanFrame EncodeCalibrationSamples(std::uint16_t samples);
std::optional<std::uint16_t> DecodeCalibrationSamples(const CanFrame& frame);
CanFrame EncodeMounting(const MountingMessage& message);
std::optional<MountingMessage> DecodeMounting(const CanFrame& frame);

}  // namespace hwemu::protocol::turret_mcb
