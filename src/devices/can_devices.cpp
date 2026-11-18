// The CAN devices: DJI motor controllers, the T-Motor AK80-9, CAN encoders
// and the turret MCB.
#include <algorithm>
#include <cmath>
#include <numbers>
#include <sstream>

#include "hwemu/devices/builtin.hpp"
#include "hwemu/protocol/can_encoder.hpp"
#include "hwemu/protocol/tmotor_can.hpp"

namespace hwemu::devices {
namespace {

constexpr double kTwoPi = 2.0 * std::numbers::pi;
constexpr double kRadPerSToRpm = 60.0 / kTwoPi;

template <typename T>
T Sat(double v) {
  return static_cast<T>(std::lround(std::clamp(
      v, static_cast<double>(std::numeric_limits<T>::min()), static_cast<double>(std::numeric_limits<T>::max())
  )));
}

std::string Context(const DeviceSpec& spec) { return spec.type + " '" + spec.name + "'"; }

double Rate(const Params& p, const std::string& key, double fallback) {
  const double hz = p.Number(key, fallback);
  if (hz < 0.0) throw std::invalid_argument("rates must not be negative");
  return hz;
}

}  // namespace

// ---- DJI motor controllers -----------------------------------------------
void DjiCanMotor::Configure(const DeviceSpec& spec) {
  SetIdentity(spec);
  const std::string ctx = Context(spec);
  const Params transport(spec.transport, ctx + " transport");
  const Params bind(spec.bind, ctx + " bind");
  const Params params(spec.params, ctx + " params");

  bus_name_ = transport.Text("bus");
  feedback_id_ = static_cast<std::uint32_t>(transport.Integer("id"));
  if (!protocol::dji_motor::IsFeedbackId(feedback_id_)) {
    throw std::invalid_argument(ctx + ": id must be a feedback id 0x201..0x20B");
  }
  const std::string mode = params.Text("mode", "current");
  if (mode == "current") {
    mode_ = sim::MotorCommand::Mode::kCurrent;
  } else if (mode == "voltage") {
    mode_ = sim::MotorCommand::Mode::kVoltage;
  } else {
    throw std::invalid_argument(ctx + ": mode must be current or voltage");
  }
  // GM6020s in current control take 0x1FE/0x2FE; everything else 0x200/0x1FF/0x2FF.
  const bool current_frame = params.Flag("gm6020_current_frame", false);
  slot_ = *protocol::dji_motor::CommandSlotFor(feedback_id_, current_frame);
  command_scale_ = params.Number("command_scale");
  max_command_ = static_cast<std::int32_t>(params.Integer("max_command", 32767));
  feedback_current_scale_ = params.Number("feedback_current_scale", command_scale_);
  reduction_ = params.Number("reduction", 1.0);
  // +1: the motor's positive turns its joint positive about the model's joint
  // axis; -1: it's mounted the other way round (what a firmware's "inverted"
  // flag compensates for).
  direction_ = params.Number("direction", 1.0) < 0 ? -1.0 : 1.0;
  // The raw encoder count the motor reports with its joint at zero: where
  // the rotor's magnet happens to sit, as a firmware's calibration records it.
  encoder_offset_ = params.Number("encoder_offset", 0.0);
  default_temperature_ = params.Number("temperature", 30.0);
  feedback_.SetRate(Rate(params, "feedback_rate_hz", 1000.0));
  watchdog_.timeout = params.Number("command_timeout", 0.1);
  motor_name_ = bind.Text("motor");
  encoder_name_ = bind.Text("encoder");
}

void DjiCanMotor::Bind(sim::ISim& sim, Transports& t) {
  motor_ = sim.Motor(motor_name_);
  encoder_ = sim.Encoder(encoder_name_);
  bus_ = t.Can(bus_name_);
  t.Router().Subscribe(bus_, slot_.frame_id, false, [this](const protocol::CanFrame& f) { OnCommand(f); });
}

void DjiCanMotor::OnCommand(const protocol::CanFrame& frame) {
  const auto raw = protocol::dji_motor::DecodeCommand(frame, slot_);
  if (!raw) return;
  last_raw_ = *raw;
  ++commands_;
  const double clamped = std::clamp<double>(*raw, -max_command_, max_command_);
  const double value = direction_ * clamped * command_scale_;
  motor_->Command({mode_, value});
  watchdog_.Fed(now_);
}

protocol::dji_motor::Feedback DjiCanMotor::MakeFeedback() const {
  const sim::EncoderReading enc = encoder_->Read();
  const sim::MotorReading mot = motor_->Read();
  protocol::dji_motor::Feedback fb;
  fb.encoder = protocol::can_encoder::AngleToCounts(
      (direction_ * enc.position * reduction_) +
          (encoder_offset_ * 2.0 * std::numbers::pi / protocol::dji_motor::kEncoderCounts),
      protocol::dji_motor::kEncoderCounts);
  fb.rpm = Sat<std::int16_t>(direction_ * enc.velocity * reduction_ * kRadPerSToRpm);
  fb.current =
      feedback_current_scale_ > 0.0 ? Sat<std::int16_t>(direction_ * mot.current / feedback_current_scale_) : 0;
  fb.temperature = Sat<std::int8_t>(mot.temperature.value_or(default_temperature_));
  return fb;
}

void DjiCanMotor::Tick(const SimTime& now) {
  now_ = now.seconds;
  if (watchdog_.Lapsed(now.seconds)) {
    motor_->Command({mode_, 0.0});
  }
  if (feedback_.Due(now)) {
    bus_->Send(protocol::dji_motor::EncodeFeedback(feedback_id_, MakeFeedback()));
    ++feedbacks_;
  }
}

std::string DjiCanMotor::Status() const {
  std::ostringstream s;
  s << "cmd=" << last_raw_ << " rx=" << commands_ << " tx=" << feedbacks_;
  return s.str();
}

// ---- T-Motor AK80-9 -------------------------------------------------------
void TmotorAk80::Configure(const DeviceSpec& spec) {
  SetIdentity(spec);
  const std::string ctx = Context(spec);
  const Params transport(spec.transport, ctx + " transport");
  const Params bind(spec.bind, ctx + " bind");
  const Params params(spec.params, ctx + " params");
  bus_name_ = transport.Text("bus");
  motor_id_ = static_cast<std::uint8_t>(transport.Integer("id"));
  reduction_ = params.Number("reduction", 9.0);
  default_temperature_ = params.Number("temperature", 30.0);
  feedback_.SetRate(Rate(params, "feedback_rate_hz", 500.0));
  watchdog_.timeout = params.Number("command_timeout", 0.1);
  motor_name_ = bind.Text("motor");
  encoder_name_ = bind.Text("encoder");
}

void TmotorAk80::Bind(sim::ISim& sim, Transports& t) {
  motor_ = sim.Motor(motor_name_);
  encoder_ = sim.Encoder(encoder_name_);
  bus_ = t.Can(bus_name_);
  using namespace protocol::tmotor;
  t.Router().Subscribe(bus_, CommandId(motor_id_, kCurrentMode), true, [this](const protocol::CanFrame& f) {
    if (auto ma = DecodeCurrentCommand(f, motor_id_)) {
      last_ma_ = std::clamp(*ma, -kMaxCurrentMilliamps, kMaxCurrentMilliamps);
      motor_->Command(sim::MotorCommand::Current(last_ma_ / 1000.0));
      watchdog_.Fed(now_);
    }
  });
  t.Router().Subscribe(bus_, CommandId(motor_id_, kOriginMode), true, [this](const protocol::CanFrame& f) {
    if (f.len >= 1 && f.data[0] == 1) {
      origin_ = encoder_->Read().position;  // "set the permanent origin" here
    }
    // data[0] == 2 asks for the origin: the next feedback frame answers.
  });
}

void TmotorAk80::Tick(const SimTime& now) {
  now_ = now.seconds;
  if (watchdog_.Lapsed(now.seconds)) {
    motor_->Command(sim::MotorCommand::Current(0.0));
  }
  if (!feedback_.Due(now)) return;
  const sim::EncoderReading enc = encoder_->Read();
  const sim::MotorReading mot = motor_->Read();
  protocol::tmotor::Feedback fb;
  // 0.1 deg of the output shaft, into int16's range (+/-3276.8 deg).
  double tenths = (enc.position - origin_) * 1800.0 / std::numbers::pi;
  tenths = std::fmod(tenths + 32768.0, 65536.0);
  if (tenths < 0) tenths += 65536.0;
  fb.position = static_cast<std::int16_t>(static_cast<std::int32_t>(std::floor(tenths)) - 32768);
  fb.speed = Sat<std::int16_t>(enc.velocity * reduction_ * kRadPerSToRpm);
  fb.current = Sat<std::int16_t>(mot.current * 100.0);
  fb.temperature = Sat<std::int8_t>(mot.temperature.value_or(default_temperature_));
  bus_->Send(protocol::tmotor::EncodeFeedback(motor_id_, fb));
}

std::string TmotorAk80::Status() const { return "cmd_mA=" + std::to_string(last_ma_); }

// ---- CAN encoders ---------------------------------------------------------
void CanEncoderDevice::Configure(const DeviceSpec& spec) {
  SetIdentity(spec);
  const std::string ctx = Context(spec);
  const Params transport(spec.transport, ctx + " transport");
  const Params bind(spec.bind, ctx + " bind");
  const Params params(spec.params, ctx + " params");
  bus_name_ = transport.Text("bus");
  id_ = static_cast<std::uint32_t>(transport.Integer("id"));
  if (id_ < protocol::can_encoder::kFirstId || id_ > protocol::can_encoder::kLastId) {
    throw std::invalid_argument(ctx + ": id must be 0x1E0..0x1E7");
  }
  counts_ = static_cast<std::uint16_t>(params.Integer("counts", protocol::can_encoder::kCounts));
  gauss_ = static_cast<std::uint16_t>(params.Integer("gauss", 0));
  reduction_ = params.Number("reduction", 1.0);
  direction_ = params.Number("direction", 1.0) < 0 ? -1.0 : 1.0;  // -1: counts run opposite the joint axis
  offset_ = params.Number("offset", 0.0);
  period_.SetRate(Rate(params, "rate_hz", 1000.0));
  encoder_name_ = bind.Text("encoder");
}

void CanEncoderDevice::Bind(sim::ISim& sim, Transports& t) {
  encoder_ = sim.Encoder(encoder_name_);
  bus_ = t.Can(bus_name_);
}

void CanEncoderDevice::Tick(const SimTime& now) {
  if (!period_.Due(now)) return;
  const double angle = (direction_ * encoder_->Read().position * reduction_) + offset_;
  if (lamprey_) {
    bus_->Send(protocol::can_encoder::EncodeLamprey(id_, protocol::can_encoder::AngleToCentidegrees(angle)));
  } else {
    bus_->Send(protocol::can_encoder::Encode(id_, {protocol::can_encoder::AngleToCounts(angle, counts_), gauss_}));
  }
}

// ---- turret MCB -----------------------------------------------------------
void TurretMcb::Configure(const DeviceSpec& spec) {
  SetIdentity(spec);
  const std::string ctx = Context(spec);
  const Params transport(spec.transport, ctx + " transport");
  const Params bind(spec.bind, ctx + " bind");
  const Params params(spec.params, ctx + " params");
  bus_name_ = transport.Text("bus");
  gyro_name_ = bind.Text("gyro");
  accel_name_ = bind.Text("accel");
  attitude_name_ = bind.Text("attitude");
  kicker_name_ = bind.Text("kicker_switch", "");
  agitator_name_ = bind.Text("agitator_switch", "");
  axis_.SetRate(Rate(params, "imu_rate_hz", 500.0));
  status_.SetRate(Rate(params, "status_rate_hz", 100.0));
  requests_.SetRate(1.0 / std::max(params.Number("request_period", 1.0), 1e-3));
  sync_.SetRate(1.0 / std::max(params.Number("sync_period", 1.0), 1e-3));
  temperature_ = params.Number("temperature", 40.0);
  calibration_time_ = params.Number("calibration_time", 1.0);
  const std::string initial = params.Text("initial_state", "calibrated");
  using protocol::turret_mcb::ImuState;
  if (initial == "calibrated") {
    state_ = ImuState::kCalibrated;
  } else if (initial == "not_calibrated") {
    state_ = ImuState::kNotCalibrated;
  } else if (initial == "calibrating") {
    state_ = ImuState::kCalibrating;
    calibrating_until_ = calibration_time_;
  } else {
    throw std::invalid_argument(ctx + ": initial_state must be calibrated, not_calibrated or calibrating");
  }
}

void TurretMcb::Bind(sim::ISim& sim, Transports& t) {
  gyro_ = sim.Gyro(gyro_name_);
  accel_ = sim.Accel(accel_name_);
  attitude_ = sim.Attitude(attitude_name_);
  if (!kicker_name_.empty()) kicker_ = sim.Switch(kicker_name_);
  if (!agitator_name_.empty()) agitator_ = sim.Switch(agitator_name_);
  bus_ = t.Can(bus_name_);
  using namespace protocol::turret_mcb;
  for (std::uint32_t id : {kCommandId, kCalibrationSamplesId, kImuMountingId, kSyncResponseId}) {
    t.Router().Subscribe(bus_, id, false, [this](const protocol::CanFrame& f) { OnFrame(f, now_); });
  }
}

void TurretMcb::OnFrame(const protocol::CanFrame& frame, double now) {
  using namespace protocol::turret_mcb;
  if (auto cmd = DecodeCommand(frame)) {
    const bool recalibrate = (*cmd & kRecalibrateImu) != 0 && (command_ & kRecalibrateImu) == 0;
    command_ = *cmd;
    if (recalibrate && state_ != ImuState::kCalibrating) {
      state_ = ImuState::kCalibrating;
      calibrating_until_ = now + calibration_time_;
    }
  } else if (auto samples = DecodeCalibrationSamples(frame)) {
    calibration_samples_ = *samples;
  } else if (auto mounting = DecodeMounting(frame)) {
    mountings_.push_back(*mounting);
    mounting_received_ = true;
  } else if (auto micros = DecodeSyncResponse(frame)) {
    chassis_micros_ = *micros;
  }
}

void TurretMcb::Tick(const SimTime& now) {
  using namespace protocol::turret_mcb;
  now_ = now.seconds;
  if (state_ == ImuState::kCalibrating && now.seconds >= calibrating_until_) {
    state_ = ImuState::kCalibrated;
  }
  // Until the chassis has answered, ask again now and then -- as the turret
  // board does after a reset.
  if (requests_.Due(now)) {
    if (!calibration_samples_) bus_->Send(EncodeRequest(kCalibrationSamplesRequestId));
    if (!mounting_received_) bus_->Send(EncodeRequest(kImuMountingRequestId));
  }
  if (sync_.Due(now)) {
    bus_->Send(EncodeRequest(kSyncRequestId));
  }
  if (axis_.Due(now) && state_ != ImuState::kCalibrating) {
    const sim::Vec3 w = gyro_->Read();
    const sim::Vec3 a = accel_->Read();
    const sim::AttitudeReading att = attitude_->Read();
    ++seq_;
    bus_->Send(EncodeAxis(kXAxisId, {AngleToFixed(att.roll), RateToRaw(w.x), AccelerationToRaw(a.x), seq_}));
    bus_->Send(EncodeAxis(kYAxisId, {AngleToFixed(att.pitch), RateToRaw(w.y), AccelerationToRaw(a.y), seq_}));
    bus_->Send(EncodeAxis(kZAxisId, {AngleToFixed(att.yaw), RateToRaw(w.z), AccelerationToRaw(a.z), seq_}));
  }
  if (status_.Due(now)) {
    StatusMessage s;
    if (kicker_ && kicker_->Read()) s.bitmask |= kKickerLimitSwitch;
    if (agitator_ && agitator_->Read()) s.bitmask |= kAgitatorLimitSwitch;
    s.imu_state = state_;
    s.temperature_centi_c = static_cast<std::int16_t>(std::lround(temperature_ * 100.0));
    bus_->Send(EncodeStatus(s));
  }
}

std::string TurretMcb::Status() const {
  std::ostringstream s;
  s << "imu_state=" << static_cast<int>(state_) << " cmd=0x" << std::hex << static_cast<int>(command_);
  return s.str();
}

}  // namespace hwemu::devices
