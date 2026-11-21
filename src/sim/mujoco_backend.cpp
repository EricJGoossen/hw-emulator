// The MuJoCo backend: the URDF (or MJCF) is parsed into an mjSpec, and every
// element devices.yaml names (SimOptions::mujoco_elements) is added to it
// before compiling -- an actuator for each motor, a site and MuJoCo sensors
// for each IMU kind, a camera for each camera. Then:
//
//   - Motors are DC motors modelled here, as Achilles's DcMotor is: a current
//     command is tracked (to the driver's limit), a voltage one sets the
//     current through the winding less the back-EMF, Kt and the gear ratio
//     give the joint torque, written as the actuator's control. The winding
//     heats through the same first-order thermal model.
//   - MuJoCo stores a sensor's `noise` but never applies it (mjModel::
//     sensor_noise is documentation for user code), so this backend adds it:
//     white Gaussian noise of that standard deviation on every sample, from
//     a generator seeded by SimOptions::seed, then any resolution.
//   - Cameras render offscreen through EGL each `period`; depth is
//     linearized to metres along the view axis.
//   - Lidars cast mj_ray fans from their site each `period`.
//   - Switches threshold their joint's position, with hysteresis.
//
// Element specs (a YAML map, name -> spec), by type:
//   motor:    joint, torque_constant, back_emf_constant (0), resistance,
//             bus_voltage, max_current, gear_ratio (1: output turns per
//             rotor turn), gearbox_efficiency (1), viscous_friction (0, N*m*s
//             at the joint), ambient_temperature (25), thermal_resistance
//             (0: no heating), thermal_time_constant (60)
//   encoder:  joint, noise (position), velocity_noise, resolution
//   gyro, accel, mag, attitude:  body, pos, quat (w x y z), noise
//   switch:   joint, press_above and/or press_below, hysteresis (0),
//             normally_closed (false), noise
//   camera:   body, pos, quat (the camera's pose with x forward, y left,
//             z up, as Achilles's), width, height, fx, fy (or fovy, deg),
//             near (0.01), far (10), period (1/30)
//   lidar:    body, pos, quat, samples (360), fov (2*pi), min_range (0.02),
//             max_range (12), period (0.1), noise
//   exclude:  body1, body2 -- fnmatch patterns; no contacts between any
//             matching pair (what a URDF can't say: an omni wheel's rollers
//             inside the chassis's convex hull)
//   joint:    joints (an fnmatch pattern), armature, damping, frictionloss --
//             set on every matching joint
#include <fnmatch.h>
#include <mujoco/mujoco.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <numbers>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "egl_context.hpp"
#include "hwemu/sim/mujoco_extras.hpp"
#include "hwemu/sim/sim.hpp"

namespace hwemu::display {
#ifdef HWEMU_WITH_DISPLAY
std::unique_ptr<IDisplay> MakeMujocoDisplay(mjModel* model, mjData* data, std::recursive_mutex* mutex);
#endif
}  // namespace hwemu::display

namespace hwemu::sim {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

struct Spec {
  std::string name, type;
  YAML::Node node;
  double Num(const char* key, double fallback) const { return node[key] ? node[key].as<double>() : fallback; }
  double Num(const char* key) const {
    if (!node[key]) throw std::invalid_argument("MuJoCo element '" + name + "' (" + type + ") needs " + key);
    return node[key].as<double>();
  }
  std::string Text(const char* key) const {
    if (!node[key]) throw std::invalid_argument("MuJoCo element '" + name + "' (" + type + ") needs " + key);
    return node[key].as<std::string>();
  }
  template <std::size_t N>
  std::array<double, N> Vec(const char* key, std::array<double, N> fallback) const {
    if (!node[key]) return fallback;
    const auto v = node[key].as<std::vector<double>>();
    if (v.size() != N) {
      throw std::invalid_argument("MuJoCo element '" + name + "': " + key + " needs " + std::to_string(N) + " numbers");
    }
    std::array<double, N> out{};
    std::copy(v.begin(), v.end(), out.begin());
    return out;
  }
};

// ---- the per-kind state, filled by MujocoSim and read through handles ------

struct MotorState {
  int actuator = -1, dof = -1;
  double kt = 0, ke = 0, resistance = 1, bus = 0, max_current = 0, gear = 1, efficiency = 1, viscous = 0;
  double ambient = 25, r_thermal = 0, tau_thermal = 60;
  MotorCommand command;
  double current = 0, torque = 0, temperature = 25;
};

struct EncoderState {
  int pos_adr = -1, vel_adr = -1;
  double noise = 0, vel_noise = 0, resolution = 0;
  EncoderReading value;
};

struct Vec3State {
  int adr = -1;
  double noise = 0;
  Vec3 value;
};

struct AttitudeState {
  int adr = -1;
  double noise = 0;
  AttitudeReading value;
};

struct SwitchState {
  int adr = -1;
  bool has_above = false, has_below = false, normally_closed = false;
  double above = 0, below = 0, hysteresis = 0, noise = 0;
  bool pressed = false;
};

struct CameraState {
  int id = -1;
  Intrinsics intrinsics;
  double near = 0.01, far = 10, period = 1.0 / 30, next = 0;
  CameraFrame frame;
  std::vector<std::uint8_t> rgb_raw;
  std::vector<float> depth_raw;
};

struct LidarState {
  int site = -1, body = -1;
  // Doesn't see the robot it's mounted on (every body under the same root
  // body): a real one looks out through gaps its robot's collision shapes
  // don't have.
  bool ignore_robot = false;
  int samples = 360;
  double fov = 2 * std::numbers::pi, min_range = 0.02, max_range = 12, period = 0.1, next = 0, noise = 0;
  LidarScan scan;
};

// Euler angles (ZYX) from a w-first quaternion.
AttitudeReading FromQuat(const double* q) {
  AttitudeReading r;
  r.orientation = {q[0], q[1], q[2], q[3]};
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  r.roll = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  r.pitch = std::asin(std::clamp(2 * (w * y - z * x), -1.0, 1.0));
  r.yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return r;
}

double Wrap(double a) {
  a = std::remainder(a, 2 * std::numbers::pi);
  return a <= -std::numbers::pi ? a + 2 * std::numbers::pi : a;
}

class MujocoSim;

template <typename State, typename Interface>
class HandleBase : public Interface {
 public:
  HandleBase(std::shared_ptr<std::recursive_mutex> m, State* s) : mutex_(std::move(m)), s_(s) {}

 protected:
  std::shared_ptr<std::recursive_mutex> mutex_;
  State* s_;
};

class Motor final : public HandleBase<MotorState, IMotor> {
 public:
  using HandleBase::HandleBase;
  void Command(const MotorCommand& c) override {
    std::lock_guard<std::recursive_mutex> l(*mutex_);
    s_->command = c;
  }
  MotorReading Read() const override {
    std::lock_guard<std::recursive_mutex> l(*mutex_);
    return {s_->current, s_->torque, s_->temperature};
  }
};

class Encoder final : public HandleBase<EncoderState, IEncoder> {
 public:
  using HandleBase::HandleBase;
  EncoderReading Read() const override {
    std::lock_guard<std::recursive_mutex> l(*mutex_);
    return s_->value;
  }
};

class Vec3Sensor final : public HandleBase<Vec3State, IVec3Sensor> {
 public:
  using HandleBase::HandleBase;
  Vec3 Read() const override {
    std::lock_guard<std::recursive_mutex> l(*mutex_);
    return s_->value;
  }
};

class Attitude final : public HandleBase<AttitudeState, IAttitude> {
 public:
  using HandleBase::HandleBase;
  AttitudeReading Read() const override {
    std::lock_guard<std::recursive_mutex> l(*mutex_);
    return s_->value;
  }
};

class Switch final : public HandleBase<SwitchState, ISwitch> {
 public:
  using HandleBase::HandleBase;
  bool Read() const override {
    std::lock_guard<std::recursive_mutex> l(*mutex_);
    return s_->pressed != s_->normally_closed;
  }
};

class Camera final : public HandleBase<CameraState, ICamera> {
 public:
  using HandleBase::HandleBase;
  Intrinsics GetIntrinsics() const override { return s_->intrinsics; }
  std::uint64_t Sequence() const override {
    std::lock_guard<std::recursive_mutex> l(*mutex_);
    return s_->frame.sequence;
  }
  bool Read(CameraFrame* out) const override {
    std::lock_guard<std::recursive_mutex> l(*mutex_);
    if (s_->frame.sequence == 0) return false;
    *out = s_->frame;
    return true;
  }
};

class Lidar final : public HandleBase<LidarState, ILidar> {
 public:
  using HandleBase::HandleBase;
  std::uint64_t Sequence() const override {
    std::lock_guard<std::recursive_mutex> l(*mutex_);
    return s_->scan.sequence;
  }
  bool Read(LidarScan* out) const override {
    std::lock_guard<std::recursive_mutex> l(*mutex_);
    if (s_->scan.sequence == 0) return false;
    *out = s_->scan;
    return true;
  }
};

class MujocoSim final : public ISim {
 public:
  explicit MujocoSim(const SimOptions& options)
      : rng_(options.seed ? *options.seed : std::random_device{}()) {
    if (options.urdf.empty()) throw std::invalid_argument("the MuJoCo backend needs models.urdf");
    char error[1024] = "";
    spec_ = mj_parseXML(options.urdf.c_str(), nullptr, error, sizeof(error));
    if (spec_ == nullptr) throw std::runtime_error("MuJoCo couldn't load " + options.urdf + ": " + error);

    std::vector<Spec> specs;
    if (options.mujoco_elements) {
      for (const auto& kv : *options.mujoco_elements) {
        Spec s{kv.first.as<std::string>(), "", kv.second};
        if (!s.node["type"]) throw std::invalid_argument("MuJoCo element '" + s.name + "' needs a type");
        s.type = s.node["type"].as<std::string>();
        specs.push_back(std::move(s));
      }
    }
    int offwidth = 0, offheight = 0;
    for (const auto& s : specs) AddToSpec(s, &offwidth, &offheight);
    if (offwidth > 0) {
      spec_->visual.global.offwidth = std::max(offwidth, spec_->visual.global.offwidth);
      spec_->visual.global.offheight = std::max(offheight, spec_->visual.global.offheight);
    }

    model_ = mj_compile(spec_, nullptr);
    if (model_ == nullptr) {
      throw std::runtime_error("MuJoCo couldn't compile " + options.urdf + ": " + mjs_getError(spec_));
    }
    data_ = mj_makeData(model_);
    mj_forward(model_, data_);
    for (const auto& s : specs) Resolve(s);
    if (!cameras_.empty()) StartRendering();
    Sample();
#ifdef HWEMU_WITH_DISPLAY
    if (options.display == DisplayMode::kWindow) {
      display_ = display::MakeMujocoDisplay(model_, data_, mutex_.get());
    }
#endif
  }

  ~MujocoSim() override {
    display_.reset();
    if (rendering_) {
      egl_->MakeCurrent();
      mjr_freeContext(&context_);
      mjv_freeScene(&scene_);
      egl_->Release();
    }
    if (data_ != nullptr) mj_deleteData(data_);
    if (model_ != nullptr) mj_deleteModel(model_);
    if (spec_ != nullptr) mj_deleteSpec(spec_);
  }

  Backend GetBackend() const override { return Backend::kMujoco; }

  void Step(double dt) override {
    std::lock_guard<std::recursive_mutex> l(*mutex_);
    const double h = model_->opt.timestep;
    const int steps = std::max(1, static_cast<int>(std::lround(dt / h)));
    for (int i = 0; i < steps; ++i) {
      for (auto& [name, m] : motors_) UpdateMotor(&m, h);
      mj_step(model_, data_);
    }
    mj_forward(model_, data_);  // sensors at the state just reached
    Sample();
  }

  Pose BodyPose(std::string_view name) override {
    const int id = mj_name2id(model_, mjOBJ_BODY, std::string(name).c_str());
    if (id < 0) throw UnknownSensor("MuJoCo: no body '" + std::string(name) + "' for a body pose");
    const mjtNum* p = data_->xpos + (3 * id);
    const mjtNum* q = data_->xquat + (4 * id);
    return {{p[0], p[1], p[2]}, {q[0], q[1], q[2], q[3]}};
  }
  double Time() const override {
    std::lock_guard<std::recursive_mutex> l(*mutex_);
    return data_->time;
  }

  MotorHandle Motor(std::string_view n) override { return Make<class Motor>(motors_, n, "motor"); }
  EncoderHandle Encoder(std::string_view n) override { return Make<class Encoder>(encoders_, n, "encoder"); }
  Vec3Handle Gyro(std::string_view n) override { return Make<Vec3Sensor>(gyros_, n, "gyro"); }
  Vec3Handle Accel(std::string_view n) override { return Make<Vec3Sensor>(accels_, n, "accel"); }
  Vec3Handle Mag(std::string_view n) override { return Make<Vec3Sensor>(mags_, n, "mag"); }
  AttitudeHandle Attitude(std::string_view n) override { return Make<class Attitude>(attitudes_, n, "attitude"); }
  CameraHandle Camera(std::string_view n) override { return Make<class Camera>(cameras_, n, "camera"); }
  LidarHandle Lidar(std::string_view n) override { return Make<class Lidar>(lidars_, n, "lidar"); }
  SwitchHandle Switch(std::string_view n) override { return Make<class Switch>(switches_, n, "switch"); }

  std::vector<SensorInfo> List() const override {
    std::vector<SensorInfo> out;
    auto add = [&](const auto& map, const char* kind) {
      for (const auto& kv : map) out.push_back({kv.first, kind});
    };
    add(motors_, "motor");
    add(encoders_, "encoder");
    add(gyros_, "gyro");
    add(accels_, "accel");
    add(mags_, "mag");
    add(attitudes_, "attitude");
    add(cameras_, "camera");
    add(lidars_, "lidar");
    add(switches_, "switch");
    return out;
  }

  display::IDisplay* Display() override { return display_.get(); }

  MujocoNative Native() { return {model_, data_, mutex_.get()}; }

 private:
  // ---- building ------------------------------------------------------------
  mjsBody* Body(const Spec& s) {
    const std::string name = s.Text("body");
    mjsBody* b = mjs_findBody(spec_, name.c_str());
    if (b == nullptr) throw std::invalid_argument("MuJoCo element '" + s.name + "': no body '" + name + "'");
    return b;
  }

  mjsSite* AddSite(const Spec& s, const std::string& name) {
    mjsSite* site = mjs_addSite(Body(s), nullptr);
    mjs_setString(site->name, name.c_str());
    const auto pos = s.Vec<3>("pos", {0, 0, 0});
    const auto quat = s.Vec<4>("quat", {1, 0, 0, 0});
    std::copy(pos.begin(), pos.end(), site->pos);
    std::copy(quat.begin(), quat.end(), site->quat);
    return site;
  }

  void AddSensor(const std::string& name, mjtSensor type, mjtObj objtype, const std::string& obj, double noise) {
    mjsSensor* sensor = mjs_addSensor(spec_);
    mjs_setString(sensor->name, name.c_str());
    sensor->type = type;
    sensor->objtype = objtype;
    mjs_setString(sensor->objname, obj.c_str());
    sensor->noise = noise;
  }

  void AddToSpec(const Spec& s, int* offwidth, int* offheight) {
    const std::string& n = s.name;
    if (s.type == "motor") {
      mjsActuator* a = mjs_addActuator(spec_, nullptr);
      mjs_setString(a->name, n.c_str());
      a->trntype = mjTRN_JOINT;
      mjs_setString(a->target, s.Text("joint").c_str());
      a->gaintype = mjGAIN_FIXED;
      a->gainprm[0] = 1;
      a->biastype = mjBIAS_NONE;
      a->dyntype = mjDYN_NONE;
      a->ctrllimited = mjLIMITED_FALSE;
      a->gear[0] = 1;
    } else if (s.type == "encoder") {
      AddSensor(n + "/pos", mjSENS_JOINTPOS, mjOBJ_JOINT, s.Text("joint"), s.Num("noise", 0));
      AddSensor(n + "/vel", mjSENS_JOINTVEL, mjOBJ_JOINT, s.Text("joint"), s.Num("velocity_noise", 0));
    } else if (s.type == "switch") {
      AddSensor(n + "/pos", mjSENS_JOINTPOS, mjOBJ_JOINT, s.Text("joint"), s.Num("noise", 0));
    } else if (s.type == "gyro" || s.type == "accel" || s.type == "mag" || s.type == "attitude") {
      AddSite(s, n + "/site");
      const mjtSensor type = s.type == "gyro"    ? mjSENS_GYRO
                             : s.type == "accel" ? mjSENS_ACCELEROMETER
                             : s.type == "mag"   ? mjSENS_MAGNETOMETER
                                                 : mjSENS_FRAMEQUAT;
      AddSensor(n, type, mjOBJ_SITE, n + "/site", s.Num("noise", 0));
    } else if (s.type == "camera") {
      mjsCamera* c = mjs_addCamera(Body(s), nullptr);
      mjs_setString(c->name, n.c_str());
      const auto pos = s.Vec<3>("pos", {0, 0, 0});
      const auto mount = s.Vec<4>("quat", {1, 0, 0, 0});
      // MuJoCo cameras look down their -z with +y up; this turns a camera
      // looking down +x with +z up into one: x_mj = -y, y_mj = z, z_mj = -x.
      const double to_mujoco[4] = {0.5, 0.5, -0.5, -0.5};
      std::copy(pos.begin(), pos.end(), c->pos);
      mju_mulQuat(c->quat, mount.data(), to_mujoco);
      const int w = static_cast<int>(s.Num("width")), h = static_cast<int>(s.Num("height"));
      const double fovy = s.node["fy"] ? 2 * std::atan(h / (2 * s.Num("fy"))) * 180 / std::numbers::pi
                                       : s.Num("fovy", 45);
      c->fovy = fovy;
      *offwidth = std::max(*offwidth, w);
      *offheight = std::max(*offheight, h);
    } else if (s.type == "lidar") {
      AddSite(s, n + "/site");
    } else if (s.type == "exclude") {
      // Contact exclusions a URDF can't express: every body matching body1
      // against every body matching body2 (fnmatch patterns).
      const auto a = Matching(mjOBJ_BODY, s.Text("body1")), b = Matching(mjOBJ_BODY, s.Text("body2"));
      if (a.empty() || b.empty()) {
        throw std::invalid_argument("MuJoCo element '" + n + "': exclude matches no " +
                                    std::string(a.empty() ? "body1 '" + s.Text("body1") : "body2 '" + s.Text("body2")) +
                                    "'");
      }
      for (const auto& x : a) {
        for (const auto& y : b) {
          if (x == y) continue;
          mjsExclude* e = mjs_addExclude(spec_);
          mjs_setString(e->bodyname1, x.c_str());
          mjs_setString(e->bodyname2, y.c_str());
        }
      }
    } else if (s.type == "joint") {
      // Joint properties a URDF can't express (armature) or that the sim
      // overrides, for every joint matching `joints` (an fnmatch pattern).
      const auto names = Matching(mjOBJ_JOINT, s.Text("joints"));
      if (names.empty()) {
        throw std::invalid_argument("MuJoCo element '" + n + "': no joint matches '" + s.Text("joints") + "'");
      }
      for (mjsElement* el = mjs_firstElement(spec_, mjOBJ_JOINT); el != nullptr; el = mjs_nextElement(spec_, el)) {
        mjsJoint* j = mjs_asJoint(el);
        if (::fnmatch(s.Text("joints").c_str(), mjs_getString(j->name), 0) != 0) continue;
        if (s.node["armature"]) j->armature = s.Num("armature");
        if (s.node["damping"]) j->damping = s.Num("damping");
        if (s.node["frictionloss"]) j->frictionloss = s.Num("frictionloss");
      }
    } else {
      throw std::invalid_argument("MuJoCo element '" + n + "': type must be motor, encoder, gyro, accel, mag, "
                                  "attitude, switch, camera, lidar, exclude or joint, not '" + s.type + "'");
    }
  }

  // The names of every element of `type` matching an fnmatch `pattern`.
  std::vector<std::string> Matching(mjtObj type, const std::string& pattern) const {
    std::vector<std::string> out;
    for (mjsElement* el = mjs_firstElement(spec_, type); el != nullptr; el = mjs_nextElement(spec_, el)) {
      const mjString* name = type == mjOBJ_BODY ? mjs_asBody(el)->name : mjs_asJoint(el)->name;
      const char* text = mjs_getString(name);
      if (text != nullptr && ::fnmatch(pattern.c_str(), text, 0) == 0) out.emplace_back(text);
    }
    return out;
  }

  int SensorAdr(const std::string& name) const {
    const int id = mj_name2id(model_, mjOBJ_SENSOR, name.c_str());
    if (id < 0) throw std::runtime_error("MuJoCo: sensor '" + name + "' missing after compile");
    return model_->sensor_adr[id];
  }
  double SensorNoise(const std::string& name) const {
    return model_->sensor_noise[mj_name2id(model_, mjOBJ_SENSOR, name.c_str())];
  }

  void Resolve(const Spec& s) {
    const std::string& n = s.name;
    if (s.type == "motor") {
      MotorState m;
      m.actuator = mj_name2id(model_, mjOBJ_ACTUATOR, n.c_str());
      const int joint = mj_name2id(model_, mjOBJ_JOINT, s.Text("joint").c_str());
      if (joint < 0) throw std::invalid_argument("MuJoCo element '" + n + "': no joint '" + s.Text("joint") + "'");
      m.dof = model_->jnt_dofadr[joint];
      m.kt = s.Num("torque_constant");
      m.ke = s.Num("back_emf_constant", 0);
      m.resistance = s.Num("resistance");
      m.bus = s.Num("bus_voltage");
      m.max_current = s.Num("max_current");
      m.gear = s.Num("gear_ratio", 1);
      m.efficiency = s.Num("gearbox_efficiency", 1);
      m.viscous = s.Num("viscous_friction", 0);
      m.ambient = s.Num("ambient_temperature", 25);
      m.r_thermal = s.Num("thermal_resistance", 0);
      m.tau_thermal = s.Num("thermal_time_constant", 60);
      m.temperature = m.ambient;
      if (m.kt <= 0 || m.resistance <= 0 || m.bus <= 0 || m.max_current <= 0 || m.gear <= 0) {
        throw std::invalid_argument("MuJoCo motor '" + n +
                                    "': torque_constant, resistance, bus_voltage, max_current and gear_ratio "
                                    "must be positive");
      }
      motors_[n] = m;
    } else if (s.type == "encoder") {
      EncoderState e;
      e.pos_adr = SensorAdr(n + "/pos");
      e.vel_adr = SensorAdr(n + "/vel");
      e.noise = SensorNoise(n + "/pos");
      e.vel_noise = SensorNoise(n + "/vel");
      e.resolution = s.Num("resolution", 0);
      encoders_[n] = e;
    } else if (s.type == "switch") {
      SwitchState w;
      w.adr = SensorAdr(n + "/pos");
      w.noise = SensorNoise(n + "/pos");
      w.has_above = static_cast<bool>(s.node["press_above"]);
      w.has_below = static_cast<bool>(s.node["press_below"]);
      if (!w.has_above && !w.has_below) {
        throw std::invalid_argument("MuJoCo switch '" + n + "' needs press_above, press_below, or both");
      }
      w.above = s.Num("press_above", 0);
      w.below = s.Num("press_below", 0);
      w.hysteresis = s.Num("hysteresis", 0);
      w.normally_closed = s.node["normally_closed"] && s.node["normally_closed"].as<bool>();
      switches_[n] = w;
    } else if (s.type == "gyro" || s.type == "accel" || s.type == "mag") {
      auto& map = s.type == "gyro" ? gyros_ : s.type == "accel" ? accels_ : mags_;
      map[n] = Vec3State{SensorAdr(n), SensorNoise(n), {}};
    } else if (s.type == "attitude") {
      attitudes_[n] = AttitudeState{SensorAdr(n), SensorNoise(n), {}};
    } else if (s.type == "camera") {
      CameraState c;
      c.id = mj_name2id(model_, mjOBJ_CAMERA, n.c_str());
      auto& in = c.intrinsics;
      in.width = static_cast<std::uint32_t>(s.Num("width"));
      in.height = static_cast<std::uint32_t>(s.Num("height"));
      const double fy = 0.5 * in.height / std::tan(model_->cam_fovy[c.id] * std::numbers::pi / 360);
      in.fy = fy;
      in.fx = s.Num("fx", fy);  // MuJoCo renders square pixels; fx is reported as given
      in.cx = s.Num("cx", 0.5 * in.width);
      in.cy = s.Num("cy", 0.5 * in.height);
      c.near = s.Num("near", 0.01);
      c.far = s.Num("far", 10);
      c.period = s.Num("period", 1.0 / 30);
      c.frame.width = in.width;
      c.frame.height = in.height;
      cameras_[n] = std::move(c);
    } else if (s.type == "lidar") {
      LidarState d;
      d.site = mj_name2id(model_, mjOBJ_SITE, (n + "/site").c_str());
      d.body = model_->site_bodyid[d.site];
      d.samples = static_cast<int>(s.Num("samples", 360));
      d.fov = s.Num("fov", 2 * std::numbers::pi);
      d.min_range = s.Num("min_range", 0.02);
      d.max_range = s.Num("max_range", 12);
      d.period = s.Num("period", 0.1);
      d.noise = s.Num("noise", 0);
      d.ignore_robot = s.node["ignore_robot"] && s.node["ignore_robot"].as<bool>();
      d.scan.min_range = static_cast<float>(d.min_range);
      d.scan.max_range = static_cast<float>(d.max_range);
      for (int i = 0; i < d.samples; ++i) {
        d.scan.angles.push_back(static_cast<float>(-0.5 * d.fov + (i + 0.5) * d.fov / d.samples));
      }
      lidars_[n] = std::move(d);
    }
  }

  void StartRendering() {
    egl_ = std::make_unique<EglContext>();
    mjv_defaultOption(&vis_options_);
    mjv_defaultScene(&scene_);
    mjv_makeScene(model_, &scene_, 2000);
    mjr_defaultContext(&context_);
    mjr_makeContext(model_, &context_, mjFONTSCALE_100);
    mjr_setBuffer(mjFB_OFFSCREEN, &context_);
    if (context_.currentBuffer != mjFB_OFFSCREEN) throw std::runtime_error("MuJoCo: no offscreen buffer");
    rendering_ = true;
    egl_->Release();
  }

  // ---- stepping ------------------------------------------------------------
  void UpdateMotor(MotorState* m, double h) {
    const double joint_speed = data_->qvel[m->dof];
    const double rotor_speed = joint_speed / m->gear;
    double current = 0;
    if (m->command.mode == MotorCommand::Mode::kCurrent) {
      current = m->command.value;
    } else {
      const double volts = std::clamp(m->command.value, -m->bus, m->bus);
      current = (volts - m->ke * rotor_speed) / m->resistance;
    }
    // The driver can't push more current than its limit, or than the bus
    // allows against the back-EMF.
    const double headroom_hi = (m->bus - m->ke * rotor_speed) / m->resistance;
    const double headroom_lo = (-m->bus - m->ke * rotor_speed) / m->resistance;
    current = std::clamp(current, std::min(headroom_lo, 0.0), std::max(headroom_hi, 0.0));
    current = std::clamp(current, -m->max_current, m->max_current);
    m->current = current;
    m->torque = (m->kt * current * m->efficiency / m->gear) - (m->viscous * joint_speed);
    data_->ctrl[m->actuator] = m->torque;
    if (m->r_thermal > 0) {
      const double target = m->ambient + current * current * m->resistance * m->r_thermal;
      m->temperature += (target - m->temperature) * (1 - std::exp(-h / m->tau_thermal));
    }
  }

  double Noise(double sigma) {
    if (sigma <= 0) return 0;
    return std::normal_distribution<double>(0, sigma)(rng_);
  }

  void Sample() {
    const double* sd = data_->sensordata;
    for (auto& [n, e] : encoders_) {
      double p = sd[e.pos_adr] + Noise(e.noise);
      if (e.resolution > 0) p = std::round(p / e.resolution) * e.resolution;
      e.value = {p, sd[e.vel_adr] + Noise(e.vel_noise)};
    }
    for (auto* map : {&gyros_, &accels_, &mags_}) {
      for (auto& [n, v] : *map) {
        v.value = {sd[v.adr] + Noise(v.noise), sd[v.adr + 1] + Noise(v.noise), sd[v.adr + 2] + Noise(v.noise)};
      }
    }
    for (auto& [n, a] : attitudes_) {
      AttitudeReading r = FromQuat(sd + a.adr);
      if (a.noise > 0) {
        r.roll = Wrap(r.roll + Noise(a.noise));
        r.pitch = r.pitch + Noise(a.noise);
        r.yaw = Wrap(r.yaw + Noise(a.noise));
      }
      a.value = r;
    }
    for (auto& [n, w] : switches_) {
      const double p = sd[w.adr] + Noise(w.noise);
      const double slack = w.pressed ? w.hysteresis : 0.0;
      w.pressed = (w.has_above && p >= w.above - slack) || (w.has_below && p <= w.below + slack);
    }
    for (auto& [n, d] : lidars_) {
      if (data_->time + 1e-9 < d.next) continue;
      d.next = data_->time + d.period;
      CastLidar(&d);
    }
    for (auto& [n, c] : cameras_) {
      if (data_->time + 1e-9 < c.next) continue;
      c.next = data_->time + c.period;
      Render(&c);
    }
  }

  // The distance along `dir` from `p` to the first geom it hits, or -1 --
  // with ignore_robot, the first not on the lidar's own robot: the ray goes
  // on past those (up to a few times; a robot is never deeper than that).
  double RayPastRobot(const LidarState* d, const double* p, const double* dir) const {
    int geom = -1;
    double travelled = 0.0;
    double from[3] = {p[0], p[1], p[2]};
    const int root = model_->body_rootid[d->body];
    for (int pass = 0; pass < 16; ++pass) {
      const double r = mj_ray(model_, data_, from, dir, nullptr, 1, d->body, &geom);
      if (r < 0) return -1.0;
      const bool own = d->ignore_robot && model_->body_rootid[model_->geom_bodyid[geom]] == root && root != 0;
      if (!own) return travelled + r;
      constexpr double kPast = 1e-4;  // m, just past the robot's surface
      travelled += r + kPast;
      for (int k = 0; k < 3; ++k) from[k] += dir[k] * (r + kPast);
    }
    return -1.0;
  }

  void CastLidar(LidarState* d) {
    const double* p = data_->site_xpos + 3 * d->site;
    const double* R = data_->site_xmat + 9 * d->site;
    d->scan.ranges.resize(static_cast<std::size_t>(d->samples));
    for (int i = 0; i < d->samples; ++i) {
      const double a = d->scan.angles[static_cast<std::size_t>(i)];
      const double local[3] = {std::cos(a), std::sin(a), 0};
      double dir[3];
      mju_mulMatVec3(dir, R, local);
      double r = RayPastRobot(d, p, dir);
      double range = (r < 0) ? kInf : r + Noise(d->noise);
      if (range < d->min_range || range > d->max_range) range = kInf;
      d->scan.ranges[static_cast<std::size_t>(i)] = static_cast<float>(range);
    }
    d->scan.capture_time = data_->time;
    ++d->scan.sequence;
  }

  void Render(CameraState* c) {
    egl_->MakeCurrent();
    mjvCamera cam;
    mjv_defaultCamera(&cam);
    cam.type = mjCAMERA_FIXED;
    cam.fixedcamid = c->id;
    const int w = static_cast<int>(c->intrinsics.width), h = static_cast<int>(c->intrinsics.height);
    const mjrRect viewport = {0, 0, w, h};
    mjv_updateScene(model_, data_, &vis_options_, nullptr, &cam, mjCAT_ALL, &scene_);
    mjr_render(viewport, &scene_, &context_);
    c->rgb_raw.resize(static_cast<std::size_t>(3 * w * h));
    c->depth_raw.resize(static_cast<std::size_t>(w * h));
    mjr_readPixels(c->rgb_raw.data(), c->depth_raw.data(), viewport, &context_);

    // OpenGL's rows run bottom to top; the depth buffer is nonlinear in
    // [0, 1] between MuJoCo's own near and far planes.
    const double extent = model_->stat.extent;
    const double znear = model_->vis.map.znear * extent, zfar = model_->vis.map.zfar * extent;
    auto& f = c->frame;
    f.rgb.resize(c->rgb_raw.size());
    f.depth.resize(c->depth_raw.size());
    for (int row = 0; row < h; ++row) {
      const int src = h - 1 - row;
      std::copy_n(c->rgb_raw.begin() + 3 * src * w, 3 * w, f.rgb.begin() + 3 * row * w);
      for (int col = 0; col < w; ++col) {
        const double d = c->depth_raw[static_cast<std::size_t>(src * w + col)];
        double z = znear / (1 - d * (1 - znear / zfar));
        if (d >= 1.0 || z < c->near || z > c->far) z = kInf;
        f.depth[static_cast<std::size_t>(row * w + col)] = static_cast<float>(z);
      }
    }
    f.capture_time = data_->time;
    ++f.sequence;
    egl_->Release();
  }

  template <typename H, typename Map>
  std::shared_ptr<H> Make(Map& map, std::string_view name, const char* kind) {
    auto it = map.find(std::string(name));
    if (it == map.end()) {
      std::string known;
      for (const auto& kv : map) known += (known.empty() ? "" : ", ") + kv.first;
      throw UnknownSensor(std::string("MuJoCo model has no ") + kind + " element named '" + std::string(name) +
                          "' (declare it under bind.mujoco in devices.yaml)" +
                          (known.empty() ? "" : "; it has: " + known));
    }
    return std::make_shared<H>(mutex_, &it->second);
  }

  std::shared_ptr<std::recursive_mutex> mutex_ = std::make_shared<std::recursive_mutex>();
  std::mt19937_64 rng_;
  mjSpec* spec_ = nullptr;
  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;

  std::map<std::string, MotorState> motors_;
  std::map<std::string, EncoderState> encoders_;
  std::map<std::string, Vec3State> gyros_, accels_, mags_;
  std::map<std::string, AttitudeState> attitudes_;
  std::map<std::string, SwitchState> switches_;
  std::map<std::string, CameraState> cameras_;
  std::map<std::string, LidarState> lidars_;

  std::unique_ptr<EglContext> egl_;
  bool rendering_ = false;
  mjvOption vis_options_{};
  mjvScene scene_{};
  mjrContext context_{};
  std::unique_ptr<display::IDisplay> display_;
};

}  // namespace

std::unique_ptr<ISim> CreateMujocoSim(const SimOptions& options) { return std::make_unique<MujocoSim>(options); }

std::optional<MujocoNative> MujocoExtras(ISim& sim) {
  auto* m = dynamic_cast<MujocoSim*>(&sim);
  if (m == nullptr) return std::nullopt;
  return m->Native();
}

}  // namespace hwemu::sim
