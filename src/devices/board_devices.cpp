// The board-level devices on pinbus: SPI IMUs and limit switches.
#include <stdexcept>

#include "hwemu/devices/builtin.hpp"

namespace hwemu::devices {

void SpiImu::Configure(const DeviceSpec& spec) {
  SetIdentity(spec);
  const std::string ctx = spec.type + " '" + spec.name + "'";
  const Params transport(spec.transport, ctx + " transport");
  const Params bind(spec.bind, ctx + " bind");
  const Params params(spec.params, ctx + " params");
  chip_ = params.Text("chip");
  registers_ = protocol::spi_imu::Make(chip_);
  if (!registers_) {
    throw std::invalid_argument(ctx + ": chip must be mpu6500, ism330dhcx, bmi088_accel or bmi088_gyro");
  }
  device_ = transport.Text("device", spec.name);
  // On a shared bus, as the hardware wires it: transfers to "bus:<bus>" reach
  // this chip while its chip-select pin is low.
  spi_bus_ = transport.Text("bus", "");
  cs_pin_ = transport.Text("cs", "");
  if (spi_bus_.empty() != cs_pin_.empty()) {
    throw std::invalid_argument(ctx + ": transport needs both bus and cs, or neither");
  }
  temperature_ = params.Number("temperature", 40.0);
  odr_.SetRate(params.Number("odr_hz", 1000.0));
  gyro_name_ = bind.Text("gyro", "");
  accel_name_ = bind.Text("accel", "");
  if (gyro_name_.empty() && accel_name_.empty()) {
    throw std::invalid_argument(ctx + ": bind needs a gyro, an accel, or both");
  }
}

void SpiImu::Bind(sim::ISim& sim, Transports& t) {
  if (!gyro_name_.empty()) gyro_ = sim.Gyro(gyro_name_);
  if (!accel_name_.empty()) accel_ = sim.Accel(accel_name_);
  auto registers = registers_;
  t.PinBus().RegisterSpi(device_, [registers](std::span<const std::uint8_t> tx) { return registers->Transfer(tx); });
  if (!spi_bus_.empty()) t.PinBus().RegisterSpiOnBus(device_, spi_bus_, cs_pin_);
}

void SpiImu::Tick(const SimTime& now) {
  if (!odr_.Due(now)) return;
  protocol::spi_imu::Sample s;
  if (gyro_) {
    const sim::Vec3 w = gyro_->Read();
    s.gyro = {w.x, w.y, w.z};
  }
  if (accel_) {
    const sim::Vec3 a = accel_->Read();
    s.accel = {a.x, a.y, a.z};
  }
  s.temperature = temperature_;
  registers_->SetSample(s);
}

void LimitSwitchDevice::Configure(const DeviceSpec& spec) {
  SetIdentity(spec);
  const std::string ctx = spec.type + " '" + spec.name + "'";
  pin_ = Params(spec.transport, ctx + " transport").Text("pin");
  switch_name_ = Params(spec.bind, ctx + " bind").Text("switch");
  active_low_ = Params(spec.params, ctx + " params").Flag("active_low", false);
}

void LimitSwitchDevice::Bind(sim::ISim& sim, Transports& t) {
  switch_ = sim.Switch(switch_name_);
  pinbus_ = &t.PinBus();
  pinbus_->SetInput(pin_, active_low_);
}

void LimitSwitchDevice::Tick(const SimTime& /*now*/) {
  const bool pressed = switch_->Read();
  pinbus_->SetInput(pin_, pressed != active_low_);
}

}  // namespace hwemu::devices
