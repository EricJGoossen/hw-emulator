// _hwemu: the hub and the sim interface, from Python. A test or a notebook
// can run the hub in-process (Hub, ticked or run on a thread), or make a sim
// directly and read its kinds (Sim). Camera frames come back as numpy arrays.
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <array>
#include <atomic>
#include <memory>
#include <thread>

#include "hwemu/config/config.hpp"
#include "hwemu/hub/hub.hpp"
#include "hwemu/sim/sim.hpp"
#include "hwemu/transport/frame_shm.hpp"

namespace py = pybind11;
using namespace hwemu;

namespace {

py::dict FrameToDict(const sim::CameraFrame& f) {
  py::dict d;
  const auto h = static_cast<py::ssize_t>(f.height), w = static_cast<py::ssize_t>(f.width);
  py::array_t<std::uint8_t> rgb({h, w, py::ssize_t{3}});
  std::copy(f.rgb.begin(), f.rgb.end(), rgb.mutable_data());
  py::array_t<float> depth({h, w});
  std::copy(f.depth.begin(), f.depth.end(), depth.mutable_data());
  d["rgb"] = rgb;
  d["depth"] = depth;
  d["capture_time"] = f.capture_time;
  d["sequence"] = f.sequence;
  return d;
}

// The hub, optionally run on a background thread (released from the GIL).
class PyHub {
 public:
  PyHub(const std::string& sim_yaml, const std::string& devices_yaml, std::optional<std::string> backend,
        std::optional<std::string> runtime_dir, bool lockstep, std::optional<std::vector<std::string>> expect,
        std::optional<double> ack_timeout, std::optional<std::string> order, std::optional<double> seconds) {
    auto config = config::LoadHubConfig(sim_yaml, devices_yaml);
    if (backend) config.sim.backend = sim::ParseBackend(*backend);
    if (runtime_dir) {
      // The lockstep socket follows the runtime dir unless sim.yaml named one.
      if (config.clock.lockstep_socket == config.runtime_dir + "/clock.sock") config.clock.lockstep_socket.clear();
      config.runtime_dir = *runtime_dir;
    }
    // As the hwemu command line's --lockstep, --expect, --ack-timeout, --order, --seconds.
    if (lockstep) config.clock.mode = clock::ClockMode::kLockstep;
    if (config.clock.mode == clock::ClockMode::kLockstep && config.clock.lockstep_socket.empty()) {
      config.clock.lockstep_socket = config.runtime_dir + "/clock.sock";
    }
    if (expect) config.clock.expected = *expect;
    if (ack_timeout) config.clock.ack_timeout = *ack_timeout;
    if (order) config.clock.order = clock::ParseLockstepOrder(*order);
    if (seconds) config.duration = *seconds;
    config.sim.display = sim::DisplayMode::kHeadless;
    hub_ = std::make_unique<hub::Hub>(std::move(config));
    hub_->Start();
  }
  ~PyHub() { Stop(); }
  bool Tick() {
    py::gil_scoped_release release;
    return hub_->Tick();
  }
  void Start() {
    if (thread_.joinable()) return;
    stop_ = false;
    done_ = false;
    thread_ = std::thread([this] {
      hub_->Run(&stop_);
      done_ = true;
    });
  }
  void Stop() {
    stop_ = true;
    if (thread_.joinable()) {
      py::gil_scoped_release release;
      thread_.join();
    }
  }
  sim::ISim& Sim() { return hub_->Sim(); }
  std::string Status() const { return hub_->StatusReport(); }
  bool Running() const { return thread_.joinable() && !done_; }

 private:
  std::unique_ptr<hub::Hub> hub_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> done_{false};
  std::thread thread_;
};

}  // namespace

PYBIND11_MODULE(_hwemu, m) {
  m.doc() = "hw-emulator: the simulation hub and its backend-neutral sim interface";

  py::class_<sim::Vec3>(m, "Vec3")
      .def_readonly("x", &sim::Vec3::x)
      .def_readonly("y", &sim::Vec3::y)
      .def_readonly("z", &sim::Vec3::z)
      .def("__repr__", [](const sim::Vec3& v) {
        return "Vec3(" + std::to_string(v.x) + ", " + std::to_string(v.y) + ", " + std::to_string(v.z) + ")";
      });
  py::class_<sim::MotorReading>(m, "MotorReading")
      .def_readonly("current", &sim::MotorReading::current)
      .def_readonly("torque", &sim::MotorReading::torque)
      .def_readonly("temperature", &sim::MotorReading::temperature);
  py::class_<sim::EncoderReading>(m, "EncoderReading")
      .def_readonly("position", &sim::EncoderReading::position)
      .def_readonly("velocity", &sim::EncoderReading::velocity);
  py::class_<sim::AttitudeReading>(m, "AttitudeReading")
      .def_readonly("roll", &sim::AttitudeReading::roll)
      .def_readonly("pitch", &sim::AttitudeReading::pitch)
      .def_readonly("yaw", &sim::AttitudeReading::yaw)
      .def_property_readonly("orientation", [](const sim::AttitudeReading& r) {
        return py::make_tuple(r.orientation.w, r.orientation.x, r.orientation.y, r.orientation.z);
      });
  py::class_<sim::Intrinsics>(m, "Intrinsics")
      .def_readonly("width", &sim::Intrinsics::width)
      .def_readonly("height", &sim::Intrinsics::height)
      .def_readonly("fx", &sim::Intrinsics::fx)
      .def_readonly("fy", &sim::Intrinsics::fy)
      .def_readonly("cx", &sim::Intrinsics::cx)
      .def_readonly("cy", &sim::Intrinsics::cy);
  py::class_<sim::SensorInfo>(m, "SensorInfo")
      .def_readonly("name", &sim::SensorInfo::name)
      .def_readonly("kind", &sim::SensorInfo::kind)
      .def("__repr__", [](const sim::SensorInfo& s) { return s.kind + ":" + s.name; });

  py::class_<sim::IMotor, sim::MotorHandle>(m, "Motor")
      .def("command_current", [](sim::IMotor& mo, double a) { mo.Command(sim::MotorCommand::Current(a)); })
      .def("command_voltage", [](sim::IMotor& mo, double v) { mo.Command(sim::MotorCommand::Voltage(v)); })
      .def("read", &sim::IMotor::Read);
  py::class_<sim::IEncoder, sim::EncoderHandle>(m, "Encoder").def("read", &sim::IEncoder::Read);
  py::class_<sim::IVec3Sensor, sim::Vec3Handle>(m, "Vec3Sensor").def("read", &sim::IVec3Sensor::Read);
  py::class_<sim::IAttitude, sim::AttitudeHandle>(m, "Attitude").def("read", &sim::IAttitude::Read);
  py::class_<sim::ISwitch, sim::SwitchHandle>(m, "Switch").def("read", &sim::ISwitch::Read);
  py::class_<sim::ICamera, sim::CameraHandle>(m, "Camera")
      .def("intrinsics", &sim::ICamera::GetIntrinsics)
      .def("sequence", &sim::ICamera::Sequence)
      .def("read", [](const sim::ICamera& c) -> py::object {
        sim::CameraFrame f;
        if (!c.Read(&f)) return py::none();
        return FrameToDict(f);
      });
  py::class_<sim::ILidar, sim::LidarHandle>(m, "Lidar")
      .def("sequence", &sim::ILidar::Sequence)
      .def("read", [](const sim::ILidar& l) -> py::object {
        sim::LidarScan s;
        if (!l.Read(&s)) return py::none();
        py::dict d;
        d["ranges"] = py::array_t<float>(static_cast<py::ssize_t>(s.ranges.size()), s.ranges.data());
        d["angles"] = py::array_t<float>(static_cast<py::ssize_t>(s.angles.size()), s.angles.data());
        d["min_range"] = s.min_range;
        d["max_range"] = s.max_range;
        d["capture_time"] = s.capture_time;
        d["sequence"] = s.sequence;
        return d;
      });

  py::class_<sim::ISim>(m, "Sim")
      .def_property_readonly("backend", [](const sim::ISim& s) { return std::string(sim::Name(s.GetBackend())); })
      .def("step", &sim::ISim::Step, py::arg("dt"), py::call_guard<py::gil_scoped_release>())
      .def("time", &sim::ISim::Time)
      .def("motor", &sim::ISim::Motor)
      .def("encoder", &sim::ISim::Encoder)
      .def("gyro", &sim::ISim::Gyro)
      .def("accel", &sim::ISim::Accel)
      .def("mag", &sim::ISim::Mag)
      .def("attitude", &sim::ISim::Attitude)
      .def("camera", &sim::ISim::Camera)
      .def("lidar", &sim::ISim::Lidar)
      .def("switch", &sim::ISim::Switch)
      .def("list", &sim::ISim::List)
      .def(
          "body_pose",
          [](sim::ISim& s, const std::string& name) {
            const sim::Pose p = s.BodyPose(name);
            py::dict d;
            d["position"] = py::make_tuple(p.position.x, p.position.y, p.position.z);
            d["orientation"] = py::make_tuple(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
            return d;
          },
          py::arg("name"), "Ground truth: a body's world pose (a MuJoCo body; an Achilles archetype/joint).")
      .def(
          "set_body_pose",
          [](sim::ISim& s, const std::string& name, std::array<double, 3> position, std::array<double, 4> orientation) {
            sim::Pose p;
            p.position = {position[0], position[1], position[2]};
            p.orientation = {orientation[0], orientation[1], orientation[2], orientation[3]};
            s.SetBodyPose(name, p);
          },
          py::arg("name"), py::arg("position"), py::arg("orientation") = std::array<double, 4>{1, 0, 0, 0},
          "Places a scripted body (a MuJoCo mocap body; an Achilles welded root joint) for the next step: "
          "position (x, y, z), orientation (w, x, y, z).");

  m.def(
      "make_sim",
      [](const std::string& sim_yaml, const std::string& devices_yaml, std::optional<std::string> backend) {
        auto config = config::LoadHubConfig(sim_yaml, devices_yaml);
        if (backend) config.sim.backend = sim::ParseBackend(*backend);
        config.sim.display = sim::DisplayMode::kHeadless;
        return sim::CreateSim(config.sim);
      },
      py::arg("sim_yaml"), py::arg("devices_yaml"), py::arg("backend") = py::none(),
      "A headless sim from a hub's sim.yaml and devices.yaml, with no devices or transports.");

  py::class_<PyHub>(m, "Hub")
      .def(py::init<const std::string&, const std::string&, std::optional<std::string>, std::optional<std::string>,
                    bool, std::optional<std::vector<std::string>>, std::optional<double>, std::optional<std::string>,
                    std::optional<double>>(),
           py::arg("sim_yaml"), py::arg("devices_yaml"), py::arg("backend") = py::none(),
           py::arg("runtime_dir") = py::none(), py::kw_only(), py::arg("lockstep") = false,
           py::arg("expect") = py::none(), py::arg("ack_timeout") = py::none(), py::arg("order") = py::none(),
           py::arg("seconds") = py::none())
      .def("tick", &PyHub::Tick, "One hub tick; False once it should stop.")
      .def("start", &PyHub::Start, "Runs the hub on a background thread.")
      .def("stop", &PyHub::Stop)
      .def_property_readonly("sim", &PyHub::Sim, py::return_value_policy::reference_internal)
      .def("status", &PyHub::Status)
      .def("running", &PyHub::Running, "True while the background thread runs (False once --seconds is up).");

  m.def("backends", [] {
    std::vector<std::string> out{"fake"};
#ifdef HWEMU_WITH_ACHILLES
    out.emplace_back("achilles");
#endif
#ifdef HWEMU_WITH_MUJOCO
    out.emplace_back("mujoco");
#endif
    return out;
  });
}
