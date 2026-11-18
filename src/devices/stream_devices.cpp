// The streaming devices: the LD19 lidar (PTY), the RealSense D435 (shared
// memory) and the MCB <-> Jetson vision link (a PTY relay).
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>

#include "hwemu/devices/builtin.hpp"
#include "hwemu/protocol/bytes.hpp"
#include "hwemu/protocol/ld19.hpp"
#include "hwemu/protocol/vision.hpp"

namespace hwemu::devices {

// ---- LD19 -----------------------------------------------------------------
void Ld19Lidar::Configure(const DeviceSpec& spec) {
  SetIdentity(spec);
  const std::string ctx = spec.type + " '" + spec.name + "'";
  const Params transport(spec.transport, ctx + " transport");
  const Params params(spec.params, ctx + " params");
  port_spec_ = transport.Text("port", spec.name);
  lidar_name_ = Params(spec.bind, ctx + " bind").Text("lidar");
  intensity_ = static_cast<std::uint8_t>(params.Integer("intensity", 200));
  scan_rate_hz_ = params.Number("scan_rate_hz", 0.0);
  min_range_ = params.Number("min_range", 0.02);
  max_range_ = params.Number("max_range", 12.0);
}

void Ld19Lidar::Bind(sim::ISim& sim, Transports& t) {
  lidar_ = sim.Lidar(lidar_name_);
  port_ = t.OpenSerial(port_spec_, protocol::ld19::kBaudRate);
}

std::string Ld19Lidar::PortPath() const { return port_ ? port_->Path() : ""; }

void Ld19Lidar::Tick(const SimTime& now) {
  std::vector<std::uint8_t> discard;
  port_->Read(&discard);  // the LD19 takes no commands
  const std::uint64_t seq = lidar_->Sequence();
  if (seq == last_sequence_) return;
  sim::LidarScan scan;
  if (!lidar_->Read(&scan)) return;
  last_sequence_ = seq;
  double rate = scan_rate_hz_;
  if (rate <= 0.0) {
    rate = last_scan_time_ >= 0.0 && scan.capture_time > last_scan_time_ ? 1.0 / (scan.capture_time - last_scan_time_)
                                                                          : 10.0;
  }
  last_scan_time_ = scan.capture_time;
  const auto ms = static_cast<std::uint64_t>(std::llround(now.seconds * 1000.0));
  for (const auto& packet : protocol::ld19::PacketsForScan(scan.ranges, scan.angles, rate, ms, intensity_, min_range_,
                                                           max_range_)) {
    const auto bytes = protocol::ld19::Encode(packet);
    port_->Write(bytes);
    ++packets_;
  }
}

std::string Ld19Lidar::Status() const { return "packets=" + std::to_string(packets_) + " port=" + PortPath(); }

// ---- RealSense D435 -------------------------------------------------------
void RealsenseD435::Configure(const DeviceSpec& spec) {
  SetIdentity(spec);
  const std::string ctx = spec.type + " '" + spec.name + "'";
  const Params transport(spec.transport, ctx + " transport");
  const Params params(spec.params, ctx + " params");
  shm_name_ = transport.Text("shm", "hwemu_" + spec.name);
  camera_name_ = Params(spec.bind, ctx + " bind").Text("camera");
  depth_scale_ = params.Number("depth_scale", 0.001);
  serial_ = params.Text("serial", "hwemu-" + spec.name);
  slots_ = static_cast<std::uint32_t>(params.Integer("slots", 4));
}

void RealsenseD435::Bind(sim::ISim& sim, Transports& t) {
  camera_ = sim.Camera(camera_name_);
  const sim::Intrinsics in = camera_->GetIntrinsics();
  const std::size_t pixels = std::size_t{in.width} * in.height;
  shm_ = std::make_unique<transport::FrameShmWriter>(shm_name_, slots_, 64, pixels * 5);
  // A sidecar so the pyrealsense2 shim knows the streams before a frame lands.
  info_path_ = t.Resolve(Name() + ".json");
  std::ofstream info(info_path_);
  info << "{\"shm\":\"" << shm_name_ << "\",\"serial\":\"" << serial_ << "\",\"width\":" << in.width
       << ",\"height\":" << in.height << ",\"fx\":" << in.fx << ",\"fy\":" << in.fy << ",\"ppx\":" << in.cx
       << ",\"ppy\":" << in.cy << ",\"depth_scale\":" << depth_scale_ << ",\"color_format\":\"rgb8\""
       << ",\"depth_format\":\"z16\"}\n";
}

void RealsenseD435::Tick(const SimTime& now) {
  const std::uint64_t seq = camera_->Sequence();
  if (seq == last_sequence_) return;
  sim::CameraFrame frame;
  if (!camera_->Read(&frame)) return;
  last_sequence_ = seq;
  const sim::Intrinsics in = camera_->GetIntrinsics();
  const std::size_t pixels = std::size_t{frame.width} * frame.height;
  depth_.resize(pixels);
  for (std::size_t i = 0; i < pixels; ++i) {
    const float d = i < frame.depth.size() ? frame.depth[i] : 0.0F;
    // z16: depth / depth_scale; 0 where there's no return (as the D435).
    if (!std::isfinite(d) || d <= 0.0F) {
      depth_[i] = 0;
    } else {
      depth_[i] = static_cast<std::uint16_t>(std::min(65535.0, std::round(d / depth_scale_)));
    }
  }
  std::array<std::uint8_t, 64> meta{};
  using protocol::PutF32Le;
  using protocol::PutU32Le;
  using protocol::PutU64Le;
  PutU32Le(&meta[0], 1);
  PutU32Le(&meta[4], frame.width);
  PutU32Le(&meta[8], frame.height);
  PutU32Le(&meta[12], 1);  // rgb8
  PutU32Le(&meta[16], 1);  // z16
  PutF32Le(&meta[20], static_cast<float>(in.fx));
  PutF32Le(&meta[24], static_cast<float>(in.fy));
  PutF32Le(&meta[28], static_cast<float>(in.cx));
  PutF32Le(&meta[32], static_cast<float>(in.cy));
  PutF32Le(&meta[36], static_cast<float>(depth_scale_));
  PutU64Le(&meta[40], frames_ + 1);
  std::memcpy(&meta[48], &frame.capture_time, sizeof(double));
  PutU64Le(&meta[56], static_cast<std::uint64_t>(std::llround(frame.capture_time * 1e6)));
  shm_->Write(meta,
              {std::span<const std::uint8_t>(frame.rgb),
               std::span(reinterpret_cast<const std::uint8_t*>(depth_.data()), depth_.size() * 2)},
              now.seconds);
  ++frames_;
}

std::string RealsenseD435::Status() const { return "frames=" + std::to_string(frames_) + " shm=" + shm_name_; }

// ---- vision link ----------------------------------------------------------
void VisionLink::Configure(const DeviceSpec& spec) {
  SetIdentity(spec);
  const std::string ctx = spec.type + " '" + spec.name + "'";
  const Params transport(spec.transport, ctx + " transport");
  mcb_spec_ = transport.Text("mcb", spec.name + "_mcb");
  otto_spec_ = transport.Text("otto", spec.name + "_otto");
  log_path_ = Params(spec.params, ctx + " params").Text("log", "");
}

void VisionLink::Bind(sim::ISim& /*sim*/, Transports& t) {
  // The MCB's UART runs at 1 Mbaud and the Jetson's at whatever it opens with;
  // a PTY doesn't care.
  mcb_ = t.OpenSerial(mcb_spec_, 1000000);
  otto_ = t.OpenSerial(otto_spec_, 1000000);
  if (!log_path_.empty()) {
    log_.open(t.Resolve(log_path_));
  }
}

std::string VisionLink::McbPath() const { return mcb_ ? mcb_->Path() : ""; }
std::string VisionLink::OttoPath() const { return otto_ ? otto_->Path() : ""; }

void VisionLink::Relay(transport::ISerialPort& from, transport::ISerialPort& to, protocol::DjiFrameParser& parser,
                       bool from_mcb, double now) {
  std::vector<std::uint8_t> bytes;
  if (from.Read(&bytes) == 0) return;
  // Bytes go through untouched -- a bad frame reaches the far side as it was
  // sent; the tap only watches.
  to.Write(bytes);
  (from_mcb ? mcb_bytes_ : otto_bytes_) += bytes.size();
  parser.Feed(bytes);
  while (auto frame = parser.Next()) {
    Message m{now, from_mcb, *frame};
    for (auto& l : listeners_) l(m);
    if (log_.is_open()) {
      log_ << "{\"t\":" << now << ",\"dir\":\"" << (from_mcb ? "mcb->otto" : "otto->mcb") << "\",\"seq\":"
           << static_cast<int>(frame->seq) << ",\"msg\":" << protocol::vision::Describe(frame->cmd, frame->data)
           << "}\n";
    }
  }
}

void VisionLink::Tick(const SimTime& now) {
  Relay(*mcb_, *otto_, from_mcb_, true, now.seconds);
  Relay(*otto_, *mcb_, from_otto_, false, now.seconds);
  if (log_.is_open()) log_.flush();
}

std::string VisionLink::Status() const {
  std::ostringstream s;
  s << "mcb->otto " << mcb_bytes_ << " B/" << from_mcb_.GetStats().frames << " frames, otto->mcb " << otto_bytes_
    << " B/" << from_otto_.GetStats().frames << " frames";
  return s.str();
}

// ---- registry -------------------------------------------------------------
void RegisterBuiltinDevices(DeviceRegistry& r) {
  r.Register("dji_can_motor", [] { return std::make_unique<DjiCanMotor>(); });
  r.Register("tmotor_ak80", [] { return std::make_unique<TmotorAk80>(); });
  r.Register("can_encoder", [] { return std::make_unique<CanEncoderDevice>(false); });
  r.Register("lamprey_encoder", [] { return std::make_unique<CanEncoderDevice>(true); });
  r.Register("turret_mcb", [] { return std::make_unique<TurretMcb>(); });
  r.Register("spi_imu", [] { return std::make_unique<SpiImu>(); });
  r.Register("limit_switch", [] { return std::make_unique<LimitSwitchDevice>(); });
  r.Register("ld19_lidar", [] { return std::make_unique<Ld19Lidar>(); });
  r.Register("realsense_d435", [] { return std::make_unique<RealsenseD435>(); });
  r.Register("vision_link", [] { return std::make_unique<VisionLink>(); });
}

}  // namespace hwemu::devices
