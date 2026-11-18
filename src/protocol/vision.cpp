#include "hwemu/protocol/vision.hpp"

#include <cstdio>
#include <sstream>

#include "hwemu/protocol/bytes.hpp"

namespace hwemu::protocol::vision {
namespace {

void Put(LeWriter& w, const Pose6& p) {
  w.F32(p.x).F32(p.y).F32(p.z).F32(p.roll).F32(p.pitch).F32(p.yaw);
}
Pose6 GetPose(LeReader& r) {
  Pose6 p;
  p.x = r.F32();
  p.y = r.F32();
  p.z = r.F32();
  p.roll = r.F32();
  p.pitch = r.F32();
  p.yaw = r.F32();
  return p;
}

std::string Num(double v) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.6g", v);
  return buf;
}

template <typename T, std::size_t N>
std::string Arr(const std::array<T, N>& a) {
  std::string out = "[";
  for (std::size_t i = 0; i < N; ++i) {
    out += (i ? "," : "") + Num(static_cast<double>(a[i]));
  }
  return out + "]";
}

std::string PoseJson(const Pose6& p) {
  return "[" + Num(p.x) + "," + Num(p.y) + "," + Num(p.z) + "," + Num(p.roll) + "," + Num(p.pitch) + "," +
         Num(p.yaw) + "]";
}

}  // namespace

const char* Name(std::uint16_t type) {
  switch (static_cast<MessageType>(type)) {
    case MessageType::kOdometry: return "odometry";
    case MessageType::kTurretAim: return "turret_aim";
    case MessageType::kRefereeRealtime: return "referee_realtime";
    case MessageType::kCompetitionResult: return "competition_result";
    case MessageType::kRefereeWarning: return "referee_warning";
    case MessageType::kRobotId: return "robot_id";
    case MessageType::kSelectNewTarget: return "select_new_target";
    case MessageType::kReboot: return "reboot";
    case MessageType::kShutdown: return "shutdown";
    case MessageType::kWorldToTurret: return "world_to_turret";
    case MessageType::kMotionStrategy: return "motion_strategy";
    case MessageType::kHealth: return "health";
    case MessageType::kAutoNavSetpoint: return "auto_nav_setpoint";
    case MessageType::kBulletsRemaining: return "bullets_remaining";
    case MessageType::kRobotOrbit: return "robot_orbit";
    case MessageType::kWorldToArducam: return "world_to_arducam";
  }
  return "unknown";
}

Direction DirectionOf(std::uint16_t type) {
  switch (static_cast<MessageType>(type)) {
    case MessageType::kOdometry:
    case MessageType::kRefereeRealtime:
    case MessageType::kCompetitionResult:
    case MessageType::kRefereeWarning:
    case MessageType::kRobotId:
    case MessageType::kSelectNewTarget:
    case MessageType::kReboot:
    case MessageType::kShutdown:
    case MessageType::kMotionStrategy:
    case MessageType::kHealth:
      return Direction::kMcbToJetson;
    case MessageType::kTurretAim:
    case MessageType::kWorldToTurret:
    case MessageType::kAutoNavSetpoint:
    case MessageType::kRobotOrbit:
    case MessageType::kWorldToArducam:
      return Direction::kJetsonToMcb;
    case MessageType::kBulletsRemaining:
      return Direction::kEither;
  }
  return Direction::kUnknown;
}

std::optional<std::size_t> FixedSize(std::uint16_t type) {
  switch (static_cast<MessageType>(type)) {
    case MessageType::kRefereeRealtime: return 12;
    case MessageType::kCompetitionResult: return 1;
    case MessageType::kRefereeWarning: return 2;
    case MessageType::kRobotId: return 1;
    case MessageType::kSelectNewTarget: return 4;
    case MessageType::kReboot: return 0;
    case MessageType::kShutdown: return 0;
    case MessageType::kMotionStrategy: return 1;
    case MessageType::kHealth: return 28;
    case MessageType::kBulletsRemaining: return 4;
    case MessageType::kWorldToArducam: return 45;
    default: return std::nullopt;
  }
}

// ---- odometry -------------------------------------------------------------
std::vector<std::uint8_t> Encode(const Odometry& m) {
  LeWriter w;
  w.U32(m.timestamp_us);
  Put(w, m.chassis);
  w.U8(static_cast<std::uint8_t>(m.turrets.size()));
  for (const auto& t : m.turrets) {
    Put(w, t);
  }
  return w.Take();
}

std::optional<Odometry> DecodeOdometry(std::span<const std::uint8_t> data) {
  LeReader r(data);
  Odometry m;
  m.timestamp_us = r.U32();
  m.chassis = GetPose(r);
  const std::uint8_t n = r.U8();
  if (!r.Ok() || r.Remaining() != static_cast<std::size_t>(n) * 24) {
    return std::nullopt;
  }
  for (std::uint8_t i = 0; i < n; ++i) {
    m.turrets.push_back(GetPose(r));
  }
  return m;
}

// ---- turret aim -----------------------------------------------------------
std::vector<std::uint8_t> Encode(std::span<const TurretAim> turrets) {
  LeWriter w;
  for (const auto& t : turrets) {
    std::uint8_t flags = 0;
    flags |= t.target ? 1U : 0U;
    flags |= t.shot_timing ? 2U : 0U;
    w.U8(flags).U32(t.timestamp_us);
    if (t.target) {
      const auto& s = *t.target;
      w.U8(s.fire_rate);
      for (float v : s.position) w.F32(v);
      for (float v : s.velocity) w.F32(v);
      for (float v : s.acceleration) w.F32(v);
      w.F32(s.theta).F32(s.omega).F32(s.radius0).F32(s.radius1);
      for (float v : s.plate_heights) w.F32(v);
      w.U8(s.icon);
    }
    if (t.shot_timing) {
      w.Bytes(*t.shot_timing);
    }
  }
  return w.Take();
}

std::optional<std::vector<TurretAim>> DecodeTurretAim(std::span<const std::uint8_t> data) {
  LeReader r(data);
  std::vector<TurretAim> out;
  while (r.Remaining() > 0) {
    TurretAim t;
    t.flags = r.U8();
    t.timestamp_us = r.U32();
    if ((t.flags & 1U) != 0) {
      TargetState s;
      s.fire_rate = r.U8();
      for (float& v : s.position) v = r.F32();
      for (float& v : s.velocity) v = r.F32();
      for (float& v : s.acceleration) v = r.F32();
      s.theta = r.F32();
      s.omega = r.F32();
      s.radius0 = r.F32();
      s.radius1 = r.F32();
      for (float& v : s.plate_heights) v = r.F32();
      s.icon = r.U8();
      t.target = s;
    }
    if ((t.flags & 2U) != 0) {
      auto bytes = r.Bytes(kShotTimingBytes);
      if (r.Ok()) {
        std::array<std::uint8_t, kShotTimingBytes> st{};
        std::copy(bytes.begin(), bytes.end(), st.begin());
        t.shot_timing = st;
      }
    }
    if (!r.Ok()) {
      return std::nullopt;
    }
    out.push_back(t);
  }
  return out;
}

// ---- auto-nav -------------------------------------------------------------
std::vector<std::uint8_t> Encode(const AutoNavSetpoint& m) {
  LeWriter w;
  w.U32(m.sequence).F32(m.speed).U32(static_cast<std::uint32_t>(m.points.size()));
  for (const auto& [x, y] : m.points) {
    w.F32(x).F32(y);
  }
  return w.Take();
}

std::optional<AutoNavSetpoint> DecodeAutoNavSetpoint(std::span<const std::uint8_t> data) {
  LeReader r(data);
  AutoNavSetpoint m;
  m.sequence = r.U32();
  m.speed = r.F32();
  const std::uint32_t n = r.U32();
  if (!r.Ok() || r.Remaining() != static_cast<std::size_t>(n) * 8) {
    return std::nullopt;
  }
  for (std::uint32_t i = 0; i < n; ++i) {
    const float x = r.F32();
    const float y = r.F32();
    m.points.emplace_back(x, y);
  }
  return m;
}

// ---- transforms -----------------------------------------------------------
std::vector<std::uint8_t> Encode(const TransformUpdate& m) {
  LeWriter w;
  for (float v : m.translation) w.F32(v);
  for (float v : m.rotation) w.F32(v);
  if (m.tag) {
    w.F32(m.tag->first).F32(m.tag->second);
  }
  w.U64(m.timestamp_us).U8(m.turret_id);
  return w.Take();
}

std::optional<TransformUpdate> DecodeTransformUpdate(std::span<const std::uint8_t> data) {
  if (data.size() != 37 && data.size() != 45) {
    return std::nullopt;
  }
  LeReader r(data);
  TransformUpdate m;
  for (float& v : m.translation) v = r.F32();
  for (float& v : m.rotation) v = r.F32();
  if (data.size() == 45) {
    const float distance = r.F32();
    const float angle = r.F32();
    m.tag = std::make_pair(distance, angle);
  }
  m.timestamp_us = r.U64();
  m.turret_id = r.U8();
  if (!r.Ok()) {
    return std::nullopt;
  }
  return m;
}

// ---- health ---------------------------------------------------------------
std::vector<std::uint8_t> Encode(const std::array<std::uint16_t, 14>& health) {
  LeWriter w;
  for (std::uint16_t hp : health) w.U16(hp);
  return w.Take();
}

std::optional<std::array<std::uint16_t, 14>> DecodeHealth(std::span<const std::uint8_t> data) {
  if (data.size() != 28) {
    return std::nullopt;
  }
  LeReader r(data);
  std::array<std::uint16_t, 14> hp{};
  for (auto& v : hp) v = r.U16();
  return hp;
}

// ---- robot orbit ----------------------------------------------------------
std::vector<std::uint8_t> Encode(std::span<const RobotOrbitEntry> orbits) {
  LeWriter w;
  for (const auto& o : orbits) {
    w.F32(o.x).F32(o.y).F32(o.z).F32(o.radius).U16(o.robot_type);
  }
  return w.Take();
}

std::optional<std::vector<RobotOrbitEntry>> DecodeRobotOrbit(std::span<const std::uint8_t> data) {
  if (data.size() % 18 != 0 || data.size() > 3 * 18) {
    return std::nullopt;
  }
  LeReader r(data);
  std::vector<RobotOrbitEntry> out;
  while (r.Remaining() > 0) {
    RobotOrbitEntry o;
    o.x = r.F32();
    o.y = r.F32();
    o.z = r.F32();
    o.radius = r.F32();
    o.robot_type = r.U16();
    out.push_back(o);
  }
  return out;
}

// ---- referee realtime -----------------------------------------------------
std::vector<std::uint8_t> Encode(const RefereeRealtime& m) {
  LeWriter w;
  w.U8(m.competition).U16(m.remaining_round_time).U64(m.unix_time).U8(m.power);
  return w.Take();
}

std::optional<RefereeRealtime> DecodeRefereeRealtime(std::span<const std::uint8_t> data) {
  if (data.size() != 12) {
    return std::nullopt;
  }
  LeReader r(data);
  RefereeRealtime m;
  m.competition = r.U8();
  m.remaining_round_time = r.U16();
  m.unix_time = r.U64();
  m.power = r.U8();
  return m;
}

// ---- describe -------------------------------------------------------------
std::string Describe(std::uint16_t type, std::span<const std::uint8_t> data) {
  std::ostringstream out;
  out << "{\"type\":\"" << Name(type) << "\",\"id\":" << type << ",\"len\":" << data.size();
  auto fail = [&]() { out << ",\"error\":\"undecodable\""; };
  const auto fixed = FixedSize(type);
  if (fixed && *fixed != data.size()) {
    fail();
    out << "}";
    return out.str();
  }
  LeReader r(data);
  switch (static_cast<MessageType>(type)) {
    case MessageType::kOdometry: {
      if (auto m = DecodeOdometry(data)) {
        out << ",\"timestamp_us\":" << m->timestamp_us << ",\"chassis\":" << PoseJson(m->chassis)
            << ",\"turrets\":[";
        for (std::size_t i = 0; i < m->turrets.size(); ++i) {
          out << (i ? "," : "") << PoseJson(m->turrets[i]);
        }
        out << "]";
      } else {
        fail();
      }
      break;
    }
    case MessageType::kTurretAim: {
      if (auto m = DecodeTurretAim(data)) {
        out << ",\"turrets\":[";
        for (std::size_t i = 0; i < m->size(); ++i) {
          const auto& t = (*m)[i];
          out << (i ? "," : "") << "{\"flags\":" << static_cast<int>(t.flags) << ",\"timestamp_us\":"
              << t.timestamp_us;
          if (t.target) {
            out << ",\"fire_rate\":" << static_cast<int>(t.target->fire_rate)
                << ",\"position\":" << Arr(t.target->position) << ",\"velocity\":" << Arr(t.target->velocity)
                << ",\"theta\":" << Num(t.target->theta) << ",\"omega\":" << Num(t.target->omega)
                << ",\"icon\":" << static_cast<int>(t.target->icon);
          }
          out << "}";
        }
        out << "]";
      } else {
        fail();
      }
      break;
    }
    case MessageType::kAutoNavSetpoint: {
      if (auto m = DecodeAutoNavSetpoint(data)) {
        out << ",\"sequence\":" << m->sequence << ",\"speed\":" << Num(m->speed) << ",\"points\":[";
        for (std::size_t i = 0; i < m->points.size(); ++i) {
          out << (i ? "," : "") << "[" << Num(m->points[i].first) << "," << Num(m->points[i].second) << "]";
        }
        out << "]";
      } else {
        fail();
      }
      break;
    }
    case MessageType::kWorldToTurret:
    case MessageType::kWorldToArducam: {
      if (auto m = DecodeTransformUpdate(data)) {
        out << ",\"translation\":" << Arr(m->translation) << ",\"rotation\":" << Arr(m->rotation)
            << ",\"timestamp_us\":" << m->timestamp_us << ",\"turret_id\":" << static_cast<int>(m->turret_id);
        if (m->tag) {
          out << ",\"tag_distance\":" << Num(m->tag->first) << ",\"tag_angle\":" << Num(m->tag->second);
        }
      } else {
        fail();
      }
      break;
    }
    case MessageType::kHealth: {
      if (auto m = DecodeHealth(data)) {
        out << ",\"hp\":" << Arr(*m);
      } else {
        fail();
      }
      break;
    }
    case MessageType::kRobotOrbit: {
      if (auto m = DecodeRobotOrbit(data)) {
        out << ",\"orbits\":[";
        for (std::size_t i = 0; i < m->size(); ++i) {
          const auto& o = (*m)[i];
          out << (i ? "," : "") << "[" << Num(o.x) << "," << Num(o.y) << "," << Num(o.z) << ","
              << Num(o.radius) << "," << o.robot_type << "]";
        }
        out << "]";
      } else {
        fail();
      }
      break;
    }
    case MessageType::kRefereeRealtime: {
      if (auto m = DecodeRefereeRealtime(data)) {
        out << ",\"game_type\":" << (m->competition >> 4U) << ",\"game_stage\":" << (m->competition & 0xFU)
            << ",\"remaining_round_time\":" << m->remaining_round_time << ",\"unix_time\":" << m->unix_time
            << ",\"power\":" << static_cast<int>(m->power);
      }
      break;
    }
    case MessageType::kRefereeWarning:
      out << ",\"level\":" << static_cast<int>(r.U8()) << ",\"robot_id\":" << static_cast<int>(r.U8());
      break;
    case MessageType::kCompetitionResult:
    case MessageType::kRobotId:
    case MessageType::kMotionStrategy:
      out << ",\"value\":" << static_cast<int>(r.U8());
      break;
    case MessageType::kSelectNewTarget:
    case MessageType::kBulletsRemaining:
      out << ",\"value\":" << r.U32();
      break;
    case MessageType::kReboot:
    case MessageType::kShutdown:
      break;
    default:
      break;
  }
  out << "}";
  return out.str();
}

}  // namespace hwemu::protocol::vision
