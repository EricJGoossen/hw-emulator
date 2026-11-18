#include <filesystem>
#include <stdexcept>

#include "hwemu/devices/device.hpp"

namespace hwemu::devices {

Transports::Transports(std::string runtime_dir) : runtime_dir_(std::move(runtime_dir)) {
  std::filesystem::create_directories(runtime_dir_);
}

Transports::~Transports() = default;

transport::PinBusServer& Transports::PinBus() {
  if (!pinbus_) {
    pinbus_ = std::make_unique<transport::PinBusServer>(Resolve("pinbus.sock"));
  }
  return *pinbus_;
}

std::string Transports::Resolve(const std::string& name) const {
  const std::filesystem::path p(name);
  return p.is_absolute() ? name : (std::filesystem::path(runtime_dir_) / p).string();
}

std::unique_ptr<transport::ISerialPort> Transports::OpenSerial(const std::string& spec, int baud) {
  if (spec.rfind("tty:", 0) == 0) {
    return transport::OpenSerialPort(spec, baud);
  }
  const std::string path = spec.rfind("pty:", 0) == 0 ? spec.substr(4) : spec;
  return std::make_unique<transport::PtyPort>(Resolve(path));
}

DeviceRegistry& DeviceRegistry::Instance() {
  static DeviceRegistry registry;
  return registry;
}

DeviceRegistry::DeviceRegistry() { RegisterBuiltinDevices(*this); }

void DeviceRegistry::Register(const std::string& type, DeviceFactory factory) {
  factories_[type] = std::move(factory);
}

std::unique_ptr<IDevice> DeviceRegistry::Make(const std::string& type) const {
  auto it = factories_.find(type);
  if (it == factories_.end()) {
    std::string known;
    for (const auto& [name, _] : factories_) known += (known.empty() ? "" : ", ") + name;
    throw std::invalid_argument("no device type '" + type + "' (known: " + known + ")");
  }
  return it->second();
}

std::vector<std::string> DeviceRegistry::Types() const {
  std::vector<std::string> out;
  for (const auto& [name, _] : factories_) out.push_back(name);
  return out;
}

void Params::Fail(const std::string& key, const std::string& what) const {
  throw std::invalid_argument(context_ + ": '" + key + "' " + what);
}

double Params::Number(const std::string& key) const {
  if (!Has(key)) Fail(key, "is required");
  try {
    return node_[key].as<double>();
  } catch (const YAML::Exception&) {
    Fail(key, "must be a number");
  }
}

double Params::Number(const std::string& key, double fallback) const { return Has(key) ? Number(key) : fallback; }

std::int64_t Params::Integer(const std::string& key) const {
  if (!Has(key)) Fail(key, "is required");
  const std::string text = node_[key].as<std::string>();
  try {
    std::size_t used = 0;
    const long long v = std::stoll(text, &used, 0);
    if (used != text.size()) Fail(key, "must be an integer, not '" + text + "'");
    return v;
  } catch (const std::logic_error&) {
    Fail(key, "must be an integer, not '" + text + "'");
  }
}

std::int64_t Params::Integer(const std::string& key, std::int64_t fallback) const {
  return Has(key) ? Integer(key) : fallback;
}

bool Params::Flag(const std::string& key, bool fallback) const {
  if (!Has(key)) return fallback;
  try {
    return node_[key].as<bool>();
  } catch (const YAML::Exception&) {
    Fail(key, "must be true or false");
  }
}

std::string Params::Text(const std::string& key) const {
  if (!Has(key)) Fail(key, "is required");
  return node_[key].as<std::string>();
}

std::string Params::Text(const std::string& key, const std::string& fallback) const {
  return Has(key) ? Text(key) : fallback;
}

}  // namespace hwemu::devices
