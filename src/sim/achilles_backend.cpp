// The Achilles backend: every hw-emulator kind is an Achilles sensor or
// actuator kind, found by name among the scene's attachments (the .arow
// declares them all). Motors are DcMotor actuators, written a current or a
// voltage; positions come from Encoder sensors; nothing here touches a joint.
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <tuple>

#include "domain/actuation/dc_motor.hpp"
#include "domain/sensing/accelerometer.hpp"
#include "domain/sensing/attitude.hpp"
#include "domain/sensing/camera.hpp"
#include "domain/sensing/encoder.hpp"
#include "domain/sensing/gyroscope.hpp"
#include "domain/sensing/lidar.hpp"
#include "domain/sensing/limit_switch.hpp"
#include "domain/sensing/magnetometer.hpp"
#include "hwemu/sim/achilles_extras.hpp"
#include "interface/attachments.hpp"
#include "interface/simulation.hpp"

namespace hwemu::display {
#ifdef HWEMU_WITH_DISPLAY
std::unique_ptr<IDisplay> MakeAchillesDisplay(achilles::interface::Simulation& sim);
achilles::display::Renderer* AchillesRenderer(IDisplay* display);
#endif
}  // namespace hwemu::display

namespace hwemu::sim {
namespace {

namespace as = achilles::domain::sensing;
namespace aa = achilles::domain::actuation;
using achilles::interface::NamedAttachments;
using achilles::interface::Simulation;

template <typename K>
using Handle = achilles::interface::AttachmentHandle<K>;

Quat FromEuler(double roll, double pitch, double yaw) {
  const double cr = std::cos(roll / 2), sr = std::sin(roll / 2);
  const double cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
  const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
  return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
          cr * cp * sy - sr * sp * cy};
}

class Motor final : public IMotor {
 public:
  explicit Motor(Handle<aa::DcMotor> h) : h_(h) {}
  void Command(const MotorCommand& c) override {
    h_.Write(c.mode == MotorCommand::Mode::kVoltage ? aa::DcMotorCommand::Voltage(static_cast<float>(c.value))
                                                    : aa::DcMotorCommand::Current(static_cast<float>(c.value)));
  }
  MotorReading Read() const override {
    const auto out = h_.Read().value;
    return {out.current, out.effort, out.temperature};
  }

 private:
  Handle<aa::DcMotor> h_;
};

class Encoder final : public IEncoder {
 public:
  explicit Encoder(Handle<as::Encoder> h) : h_(h) {}
  EncoderReading Read() const override {
    const auto r = h_.Read().value;
    return {r.position, r.velocity};
  }

 private:
  Handle<as::Encoder> h_;
};

template <typename K>
class Vec3Sensor final : public IVec3Sensor {
 public:
  explicit Vec3Sensor(Handle<K> h) : h_(h) {}
  Vec3 Read() const override {
    const auto v = h_.Read().value;
    return {v.X(), v.Y(), v.Z()};
  }

 private:
  Handle<K> h_;
};

class Attitude final : public IAttitude {
 public:
  explicit Attitude(Handle<as::Attitude> h) : h_(h) {}
  AttitudeReading Read() const override {
    const auto v = h_.Read().value;
    AttitudeReading r;
    r.roll = v.X();
    r.pitch = v.Y();
    r.yaw = v.Z();
    r.orientation = FromEuler(r.roll, r.pitch, r.yaw);
    return r;
  }

 private:
  Handle<as::Attitude> h_;
};

class Camera final : public ICamera {
 public:
  explicit Camera(Handle<as::Camera> h) : h_(h) {}
  Intrinsics GetIntrinsics() const override {
    const auto config = h_.Config();
    const auto in = config.GetIntrinsics();
    return {config.GetParams().width, config.GetParams().height, in.fx, in.fy, in.cx, in.cy};
  }
  std::uint64_t Sequence() const override {
    const auto r = h_.Read();
    // A pipelined camera's first sample has no image yet (width 0).
    return r.value.width == 0 ? 0 : r.sequence;
  }
  bool Read(CameraFrame* out) const override {
    const auto r = h_.Read();
    const auto& f = r.value;
    if (f.width == 0) return false;
    const auto& image = h_.State().images[f.image];
    const std::size_t pixels = std::size_t{f.width} * f.height;
    out->width = f.width;
    out->height = f.height;
    out->rgb.assign(image.rgb.begin(), image.rgb.begin() + static_cast<std::ptrdiff_t>(3 * pixels));
    out->depth.assign(image.depth.begin(), image.depth.begin() + static_cast<std::ptrdiff_t>(pixels));
    out->capture_time = f.capture_time;
    out->sequence = r.sequence;
    return true;
  }

 private:
  Handle<as::Camera> h_;
};

class Lidar final : public ILidar {
 public:
  explicit Lidar(Handle<as::Lidar> h) : h_(h) {
    const auto& p = h_.Config().GetParams();
    const float step = p.horizontal_fov / static_cast<float>(p.horizontal_samples);
    for (std::uint32_t c = 0; c < p.channels; ++c) {
      for (std::uint32_t s = 0; s < p.horizontal_samples; ++s) {
        angles_.push_back((-0.5F * p.horizontal_fov) + ((static_cast<float>(s) + 0.5F) * step));
      }
    }
    min_range_ = p.min_range;
    max_range_ = p.max_range;
  }
  std::uint64_t Sequence() const override { return h_.Read().sequence; }
  bool Read(LidarScan* out) const override {
    const auto r = h_.Read();
    if (r.sequence == 0) return false;
    out->ranges.assign(r.value.ranges.begin(), r.value.ranges.begin() + r.value.beams);
    out->angles.assign(angles_.begin(), angles_.begin() + r.value.beams);
    out->min_range = min_range_;
    out->max_range = max_range_;
    out->capture_time = r.sample_time;
    out->sequence = r.sequence;
    return true;
  }

 private:
  Handle<as::Lidar> h_;
  std::vector<float> angles_;
  float min_range_ = 0, max_range_ = 0;
};

class Switch final : public ISwitch {
 public:
  explicit Switch(Handle<as::LimitSwitch> h) : h_(h) {}
  bool Read() const override { return h_.Read().value.pressed; }

 private:
  Handle<as::LimitSwitch> h_;
};

achilles::algorithms::Integrator ParseIntegrator(const std::string& name) {
  using I = achilles::algorithms::Integrator;
  if (name.empty() || name == "semi_implicit_euler" || name == "euler") return I::kSemiImplicitEuler;
  if (name == "verlet" || name == "velocity_verlet") return I::kVelocityVerlet;
  if (name == "midpoint" || name == "implicit_midpoint") return I::kImplicitMidpoint;
  if (name == "rk4" || name == "runge_kutta4") return I::kRungeKutta4;
  if (name == "implicit_fast") return I::kImplicitFast;
  throw std::invalid_argument("unknown Achilles integrator '" + name + "'");
}

class AchillesSim final : public ISim {
 public:
  explicit AchillesSim(const SimOptions& options) {
    if (options.arow.empty()) {
      throw std::invalid_argument("the Achilles backend needs models.arow");
    }
    if (!sim_.LoadArowFile(options.arow)) {
      throw std::runtime_error("Achilles couldn't load " + options.arow);
    }
    if (!options.sim_config.empty() && !sim_.LoadConfigFile(options.sim_config)) {
      throw std::runtime_error("Achilles couldn't load " + options.sim_config);
    }
    sim_.SetIntegrator(ParseIntegrator(options.integrator));
    if (!sim_.Init(options.seed)) {
      throw std::runtime_error("Achilles couldn't build the sim from " + options.arow);
    }
    lists_ = sim_.Attachments();
#ifdef HWEMU_WITH_DISPLAY
    if (options.display == DisplayMode::kWindow) {
      display_ = display::MakeAchillesDisplay(sim_);
    }
#endif
  }

  Backend GetBackend() const override { return Backend::kAchilles; }

  void Step(double dt) override {
    if (!sim_.Step(static_cast<float>(dt))) {
      throw std::runtime_error("Achilles step failed at t=" + std::to_string(sim_.Time()));
    }
  }
  double Time() const override { return sim_.Time(); }

  MotorHandle Motor(std::string_view name) override {
    return std::make_shared<class Motor>(Find<aa::DcMotor>(name, "motor"));
  }
  EncoderHandle Encoder(std::string_view name) override {
    return std::make_shared<class Encoder>(Find<as::Encoder>(name, "encoder"));
  }
  Vec3Handle Gyro(std::string_view name) override {
    return std::make_shared<Vec3Sensor<as::Gyroscope>>(Find<as::Gyroscope>(name, "gyro"));
  }
  Vec3Handle Accel(std::string_view name) override {
    return std::make_shared<Vec3Sensor<as::Accelerometer>>(Find<as::Accelerometer>(name, "accel"));
  }
  Vec3Handle Mag(std::string_view name) override {
    return std::make_shared<Vec3Sensor<as::Magnetometer>>(Find<as::Magnetometer>(name, "mag"));
  }
  AttitudeHandle Attitude(std::string_view name) override {
    return std::make_shared<class Attitude>(Find<as::Attitude>(name, "attitude"));
  }
  CameraHandle Camera(std::string_view name) override {
    return std::make_shared<class Camera>(Find<as::Camera>(name, "camera"));
  }
  LidarHandle Lidar(std::string_view name) override {
    return std::make_shared<class Lidar>(Find<as::Lidar>(name, "lidar"));
  }
  SwitchHandle Switch(std::string_view name) override {
    return std::make_shared<class Switch>(Find<as::LimitSwitch>(name, "switch"));
  }

  std::vector<SensorInfo> List() const override {
    std::vector<SensorInfo> out;
    auto add = [&]<typename K>(const char* kind) {
      for (const auto& a : std::get<NamedAttachments<K>>(lists_)) out.push_back({a.name, kind});
    };
    add.template operator()<aa::DcMotor>("motor");
    add.template operator()<as::Encoder>("encoder");
    add.template operator()<as::Gyroscope>("gyro");
    add.template operator()<as::Accelerometer>("accel");
    add.template operator()<as::Magnetometer>("mag");
    add.template operator()<as::Attitude>("attitude");
    add.template operator()<as::Camera>("camera");
    add.template operator()<as::Lidar>("lidar");
    add.template operator()<as::LimitSwitch>("switch");
    return out;
  }

  display::IDisplay* Display() override { return display_.get(); }

  Simulation& Native() { return sim_; }

 private:
  // The attachment of kind K named `name` -- instance 0, or "name#i" for
  // instance i of an archetype instanced more than once.
  template <typename K>
  Handle<K> Find(std::string_view name, const char* kind) const {
    std::string base(name);
    std::size_t instance = 0;
    if (auto hash = base.rfind('#'); hash != std::string::npos) {
      instance = std::stoul(base.substr(hash + 1));
      base = base.substr(0, hash);
    }
    for (const auto& a : std::get<NamedAttachments<K>>(lists_)) {
      if (a.name == base && a.instance == instance) return a.handle;
    }
    std::string known;
    for (const auto& a : std::get<NamedAttachments<K>>(lists_)) known += (known.empty() ? "" : ", ") + a.name;
    throw UnknownSensor(std::string("Achilles scene has no ") + kind + " (" + std::string(K::kListName) + ") named '" +
                        std::string(name) + "'" + (known.empty() ? "" : " (it has: " + known + ")"));
  }

  Simulation sim_;
  achilles::interface::AttachmentLists lists_;
  std::unique_ptr<display::IDisplay> display_;
};

}  // namespace

std::unique_ptr<ISim> CreateAchillesSim(const SimOptions& options) { return std::make_unique<AchillesSim>(options); }

std::optional<AchillesNative> AchillesExtras(ISim& sim) {
  auto* a = dynamic_cast<AchillesSim*>(&sim);
  if (a == nullptr) return std::nullopt;
  AchillesNative n;
  n.simulation = &a->Native();
#ifdef HWEMU_WITH_DISPLAY
  n.renderer = display::AchillesRenderer(a->Display());
#endif
  return n;
}

}  // namespace hwemu::sim
