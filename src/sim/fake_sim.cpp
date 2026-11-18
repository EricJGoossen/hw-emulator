#include "hwemu/sim/fake_sim.hpp"

#include <functional>

namespace hwemu::sim {
namespace {

class FakeMotor final : public IMotor {
 public:
  void Command(const MotorCommand& c) override {
    std::lock_guard<std::mutex> l(m);
    last = c;
    ++count;
  }
  MotorReading Read() const override {
    std::lock_guard<std::mutex> l(m);
    return reading;
  }
  mutable std::mutex m;
  MotorCommand last;
  int count = 0;
  MotorReading reading;
};

template <typename Interface, typename Value>
class FakeValue final : public Interface {
 public:
  Value Read() const override {
    std::lock_guard<std::mutex> l(m);
    return value;
  }
  void Set(const Value& v) {
    std::lock_guard<std::mutex> l(m);
    value = v;
  }
  mutable std::mutex m;
  Value value{};
};

class FakeCamera final : public ICamera {
 public:
  Intrinsics GetIntrinsics() const override {
    std::lock_guard<std::mutex> l(m);
    return intrinsics;
  }
  std::uint64_t Sequence() const override {
    std::lock_guard<std::mutex> l(m);
    return frame.sequence;
  }
  bool Read(CameraFrame* out) const override {
    std::lock_guard<std::mutex> l(m);
    if (frame.sequence == 0) {
      return false;
    }
    *out = frame;
    return true;
  }
  mutable std::mutex m;
  CameraFrame frame;
  Intrinsics intrinsics;
};

class FakeLidar final : public ILidar {
 public:
  std::uint64_t Sequence() const override {
    std::lock_guard<std::mutex> l(m);
    return scan.sequence;
  }
  bool Read(LidarScan* out) const override {
    std::lock_guard<std::mutex> l(m);
    if (scan.sequence == 0) {
      return false;
    }
    *out = scan;
    return true;
  }
  mutable std::mutex m;
  LidarScan scan;
};

using FakeEncoder = FakeValue<IEncoder, EncoderReading>;
using FakeVec3 = FakeValue<IVec3Sensor, Vec3>;
using FakeAttitude = FakeValue<IAttitude, AttitudeReading>;
using FakeSwitch = FakeValue<ISwitch, bool>;

}  // namespace

struct FakeSim::State {
  std::mutex m;
  std::map<std::string, std::shared_ptr<FakeMotor>, std::less<>> motors;
  std::map<std::string, std::shared_ptr<FakeEncoder>, std::less<>> encoders;
  std::map<std::string, std::shared_ptr<FakeVec3>, std::less<>> gyros, accels, mags;
  std::map<std::string, std::shared_ptr<FakeAttitude>, std::less<>> attitudes;
  std::map<std::string, std::shared_ptr<FakeCamera>, std::less<>> cameras;
  std::map<std::string, std::shared_ptr<FakeLidar>, std::less<>> lidars;
  std::map<std::string, std::shared_ptr<FakeSwitch>, std::less<>> switches;

  template <typename T>
  std::shared_ptr<T> Get(std::map<std::string, std::shared_ptr<T>, std::less<>>& map, std::string_view name) {
    std::lock_guard<std::mutex> l(m);
    auto it = map.find(name);
    if (it == map.end()) {
      it = map.emplace(std::string(name), std::make_shared<T>()).first;
    }
    return it->second;
  }
};

FakeSim::FakeSim() : state_(std::make_unique<State>()) {}
FakeSim::~FakeSim() = default;

MotorHandle FakeSim::Motor(std::string_view n) { return state_->Get(state_->motors, n); }
EncoderHandle FakeSim::Encoder(std::string_view n) { return state_->Get(state_->encoders, n); }
Vec3Handle FakeSim::Gyro(std::string_view n) { return state_->Get(state_->gyros, n); }
Vec3Handle FakeSim::Accel(std::string_view n) { return state_->Get(state_->accels, n); }
Vec3Handle FakeSim::Mag(std::string_view n) { return state_->Get(state_->mags, n); }
AttitudeHandle FakeSim::Attitude(std::string_view n) { return state_->Get(state_->attitudes, n); }
CameraHandle FakeSim::Camera(std::string_view n) { return state_->Get(state_->cameras, n); }
LidarHandle FakeSim::Lidar(std::string_view n) { return state_->Get(state_->lidars, n); }
SwitchHandle FakeSim::Switch(std::string_view n) { return state_->Get(state_->switches, n); }

std::vector<SensorInfo> FakeSim::List() const {
  std::lock_guard<std::mutex> l(state_->m);
  std::vector<SensorInfo> out;
  auto add = [&](const auto& map, const char* kind) {
    for (const auto& [name, _] : map) {
      out.push_back({name, kind});
    }
  };
  add(state_->motors, "motor");
  add(state_->encoders, "encoder");
  add(state_->gyros, "gyro");
  add(state_->accels, "accel");
  add(state_->mags, "mag");
  add(state_->attitudes, "attitude");
  add(state_->cameras, "camera");
  add(state_->lidars, "lidar");
  add(state_->switches, "switch");
  return out;
}

void FakeSim::SetMotorReading(std::string_view n, const MotorReading& r) {
  auto m = state_->Get(state_->motors, n);
  std::lock_guard<std::mutex> l(m->m);
  m->reading = r;
}
MotorCommand FakeSim::LastCommand(std::string_view n) {
  auto m = state_->Get(state_->motors, n);
  std::lock_guard<std::mutex> l(m->m);
  return m->last;
}
int FakeSim::CommandCount(std::string_view n) {
  auto m = state_->Get(state_->motors, n);
  std::lock_guard<std::mutex> l(m->m);
  return m->count;
}
void FakeSim::SetEncoder(std::string_view n, const EncoderReading& r) { state_->Get(state_->encoders, n)->Set(r); }
void FakeSim::SetGyro(std::string_view n, const Vec3& v) { state_->Get(state_->gyros, n)->Set(v); }
void FakeSim::SetAccel(std::string_view n, const Vec3& v) { state_->Get(state_->accels, n)->Set(v); }
void FakeSim::SetMag(std::string_view n, const Vec3& v) { state_->Get(state_->mags, n)->Set(v); }
void FakeSim::SetAttitude(std::string_view n, const AttitudeReading& r) {
  state_->Get(state_->attitudes, n)->Set(r);
}
void FakeSim::SetCamera(std::string_view n, const CameraFrame& f, const Intrinsics& i) {
  auto c = state_->Get(state_->cameras, n);
  std::lock_guard<std::mutex> l(c->m);
  c->frame = f;
  c->intrinsics = i;
}
void FakeSim::SetLidar(std::string_view n, const LidarScan& s) {
  auto c = state_->Get(state_->lidars, n);
  std::lock_guard<std::mutex> l(c->m);
  c->scan = s;
}
void FakeSim::SetSwitch(std::string_view n, bool pressed) { state_->Get(state_->switches, n)->Set(pressed); }

}  // namespace hwemu::sim
