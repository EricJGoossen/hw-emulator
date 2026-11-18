#pragma once

// The MCB <-> Jetson (project-otto) vision link's messages, carried in DJI
// serial frames (dji_serial.hpp) whose cmd id is the message type. Layouts
// match the code on both ends -- project_otto/comms/messages/*.py and
// aruwsrc/communication/serial/vision_coprocessor.* -- not the docs; all
// little-endian, packed:
//
//  id    dir  name                 layout
//  0x01  M>J  odometry             u32 us | 6f chassis xyz rpy | u8 n | n x 6f turret
//  0x02  J>M  turret aim           per turret: u8 flags | u32 us
//                                  | flags&1: u8 fire rate, 17f, u8 icon (70 B)
//                                  | flags&2: 12 B shot timing
//  0x03  M>J  referee realtime     u8 | u16 | u64 | u8
//  0x04  M>J  competition result   u8
//  0x05  M>J  referee warning      u8 level | u8 robot id
//  0x06  M>J  robot id             u8
//  0x07  M>J  select new target    u32 request id
//  0x08  M>J  reboot               (empty)
//  0x09  M>J  shutdown             (empty)
//  0x0A  J>M  world->turret        3f xyz | 4f quat | u64 us | u8 turret   (37 B, otto)
//                                  aruwsrc reads 0x0A as the 45 B aruco packet below
//  0x0B  M>J  motion strategy      u8
//  0x0C  M>J  health               14 x u16 (red 1..7, blue 1..7)
//  0x0D  J>M  auto-nav setpoints   u32 seq | f speed | u32 n | n x 2f
//  0x0E  both bullets remaining    u32 (otto parses it, aruwsrc lists it as rx)
//  0x0F  J>M  robot orbit          up to 3 x (4f, u16)
//  0x11  J>M  world->arducam       3f | 4f | f tag distance | f tag angle | u64 us | u8 turret

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace hwemu::protocol::vision {

enum class MessageType : std::uint16_t {
  kOdometry = 0x01,
  kTurretAim = 0x02,
  kRefereeRealtime = 0x03,
  kCompetitionResult = 0x04,
  kRefereeWarning = 0x05,
  kRobotId = 0x06,
  kSelectNewTarget = 0x07,
  kReboot = 0x08,
  kShutdown = 0x09,
  kWorldToTurret = 0x0A,
  kMotionStrategy = 0x0B,
  kHealth = 0x0C,
  kAutoNavSetpoint = 0x0D,
  kBulletsRemaining = 0x0E,
  kRobotOrbit = 0x0F,
  kWorldToArducam = 0x11,
};

enum class Direction { kMcbToJetson, kJetsonToMcb, kEither, kUnknown };

// "odometry", "turret_aim", ... or "unknown".
const char* Name(std::uint16_t type);
Direction DirectionOf(std::uint16_t type);

struct Pose6 {
  float x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;
  friend bool operator==(const Pose6&, const Pose6&) = default;
};

struct Odometry {
  std::uint32_t timestamp_us = 0;
  Pose6 chassis;
  std::vector<Pose6> turrets;
  friend bool operator==(const Odometry&, const Odometry&) = default;
};

struct TargetState {
  std::uint8_t fire_rate = 0;
  std::array<float, 3> position{}, velocity{}, acceleration{};
  float theta = 0, omega = 0, radius0 = 0, radius1 = 0;
  std::array<float, 4> plate_heights{};
  std::uint8_t icon = 0;
  friend bool operator==(const TargetState&, const TargetState&) = default;
};

struct TurretAim {
  std::uint8_t flags = 0;  // bit0 target, bit1 shot timing
  std::uint32_t timestamp_us = 0;
  std::optional<TargetState> target;
  std::optional<std::array<std::uint8_t, 12>> shot_timing;
  friend bool operator==(const TurretAim&, const TurretAim&) = default;
};

struct AutoNavSetpoint {
  std::uint32_t sequence = 0;
  float speed = 0;
  std::vector<std::pair<float, float>> points;
  friend bool operator==(const AutoNavSetpoint&, const AutoNavSetpoint&) = default;
};

// 0x0A and 0x11: a transform with a timestamp; the aruco variants (0x11, and
// 0x0A as aruwsrc reads it) carry the tag's distance and angle too.
struct TransformUpdate {
  std::array<float, 3> translation{};
  std::array<float, 4> rotation{};  // as sent: w, x, y, z
  std::optional<std::pair<float, float>> tag;  // distance, angle
  std::uint64_t timestamp_us = 0;
  std::uint8_t turret_id = 0;
  friend bool operator==(const TransformUpdate&, const TransformUpdate&) = default;
};

struct RobotOrbitEntry {
  float x = 0, y = 0, z = 0, radius = 0;
  std::uint16_t robot_type = 0;
  friend bool operator==(const RobotOrbitEntry&, const RobotOrbitEntry&) = default;
};

struct RefereeRealtime {
  std::uint8_t competition = 0;  // u4 type | u4 stage, as bitstruct ">u4u4"
  std::uint16_t remaining_round_time = 0;
  std::uint64_t unix_time = 0;
  std::uint8_t power = 0;
  friend bool operator==(const RefereeRealtime&, const RefereeRealtime&) = default;
};

inline constexpr std::size_t kTargetStateBytes = 70;
inline constexpr std::size_t kShotTimingBytes = 12;

std::vector<std::uint8_t> Encode(const Odometry& m);
std::optional<Odometry> DecodeOdometry(std::span<const std::uint8_t> data);

std::vector<std::uint8_t> Encode(std::span<const TurretAim> turrets);
std::optional<std::vector<TurretAim>> DecodeTurretAim(std::span<const std::uint8_t> data);

std::vector<std::uint8_t> Encode(const AutoNavSetpoint& m);
std::optional<AutoNavSetpoint> DecodeAutoNavSetpoint(std::span<const std::uint8_t> data);

std::vector<std::uint8_t> Encode(const TransformUpdate& m);
std::optional<TransformUpdate> DecodeTransformUpdate(std::span<const std::uint8_t> data);

std::vector<std::uint8_t> Encode(const std::array<std::uint16_t, 14>& health);
std::optional<std::array<std::uint16_t, 14>> DecodeHealth(std::span<const std::uint8_t> data);

std::vector<std::uint8_t> Encode(std::span<const RobotOrbitEntry> orbits);
std::optional<std::vector<RobotOrbitEntry>> DecodeRobotOrbit(std::span<const std::uint8_t> data);

std::vector<std::uint8_t> Encode(const RefereeRealtime& m);
std::optional<RefereeRealtime> DecodeRefereeRealtime(std::span<const std::uint8_t> data);

// The size a fixed-length message must have, or nullopt for a variable one.
std::optional<std::size_t> FixedSize(std::uint16_t type);

// One line of JSON describing a message: {"type":"odometry","id":1,...},
// with the decoded fields when it decodes, "error" when it doesn't.
std::string Describe(std::uint16_t type, std::span<const std::uint8_t> data);

}  // namespace hwemu::protocol::vision
