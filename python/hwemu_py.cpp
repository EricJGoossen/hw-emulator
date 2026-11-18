// _hwemu: the hub and the sim interface, from Python. A test or a notebook
// can run the hub in-process (Hub, ticked or run on a thread), or make a sim
// directly and read its kinds (Sim). Camera frames come back as numpy arrays.
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

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
        std::optional<std::string> runtime_dir) {
    auto config = config::LoadHubConfig(sim_yaml, devices_yaml);
    if (backend) config.sim.backend = sim::ParseBackend(*backend);
    if (runtime_dir) config.runtime_dir = *runtime_dir;
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
    thread_ = std::thread([this] { hub_->Run(&stop_); });
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

 private:
  std::unique_ptr<hub::Hub> hub_;
  std::atomic<bool> stop_{false};
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
      .def("list", &sim::ISim::List);

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
      .def(py::init<const std::string&, const std::string&, std::optional<std::string>, std::optional<std::string>>(),
           py::arg("sim_yaml"), py::arg("devices_yaml"), py::arg("backend") = py::none(),
           py::arg("runtime_dir") = py::none())
      .def("tick", &PyHub::Tick, "One hub tick; False once it should stop.")
      .def("start", &PyHub::Start, "Runs the hub on a background thread.")
      .def("stop", &PyHub::Stop)
      .def_property_readonly("sim", &PyHub::Sim, py::return_value_policy::reference_internal)
      .def("status", &PyHub::Status);

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
