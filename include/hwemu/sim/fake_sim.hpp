#pragma once

// A sim with no physics: every sensor reads what a test Set*()s, and every
// motor remembers what it was last commanded. For the device unit tests, the
// Python tests and the hub's smoke test -- anything that checks unit
// conversions and wiring without a physics engine or a GPU. Lookups create
// what they're asked for, so any name works.

#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "hwemu/sim/sim.hpp"

namespace hwemu::sim {

class FakeSim final : public ISim {
 public:
  FakeSim();
  ~FakeSim() override;

  Backend GetBackend() const override { return Backend::kFake; }
  void Step(double dt) override { time_ += dt; }
  double Time() const override { return time_; }

  MotorHandle Motor(std::string_view name) override;
  EncoderHandle Encoder(std::string_view name) override;
  Vec3Handle Gyro(std::string_view name) override;
  Vec3Handle Accel(std::string_view name) override;
  Vec3Handle Mag(std::string_view name) override;
  AttitudeHandle Attitude(std::string_view name) override;
  CameraHandle Camera(std::string_view name) override;
  LidarHandle Lidar(std::string_view name) override;
  SwitchHandle Switch(std::string_view name) override;
  std::vector<SensorInfo> List() const override;
  display::IDisplay* Display() override { return nullptr; }

  // What a test sets and reads back.
  void SetMotorReading(std::string_view name, const MotorReading& reading);
  MotorCommand LastCommand(std::string_view name);
  int CommandCount(std::string_view name);
  void SetEncoder(std::string_view name, const EncoderReading& reading);
  void SetGyro(std::string_view name, const Vec3& v);
  void SetAccel(std::string_view name, const Vec3& v);
  void SetMag(std::string_view name, const Vec3& v);
  void SetAttitude(std::string_view name, const AttitudeReading& reading);
  void SetCamera(std::string_view name, const CameraFrame& frame, const Intrinsics& intrinsics);
  void SetLidar(std::string_view name, const LidarScan& scan);
  void SetSwitch(std::string_view name, bool pressed);

  struct State;

 private:
  double time_ = 0.0;
  std::unique_ptr<State> state_;
};

}  // namespace hwemu::sim
