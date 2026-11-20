// The vision link relay over real PTYs, the frame ring across a fork, and
// the lockstep clock with participants on the client library.
#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <thread>

#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include "hwemu/clock/sim_clock.hpp"
#include "hwemu/devices/builtin.hpp"
#include "hwemu/protocol/dji_serial.hpp"
#include "hwemu/protocol/vision.hpp"
#include "hwemu/sim/fake_sim.hpp"
#include "hwemu/transport/frame_shm.hpp"
#include "hwemu_client.h"

using namespace hwemu;

namespace {
std::string TempDir(const std::string& name) {
  auto dir = std::filesystem::temp_directory_path() / ("hwemu_test_" + name + "_" + std::to_string(::getpid()));
  std::filesystem::create_directories(dir);
  return dir.string();
}

std::vector<std::uint8_t> ReadAll(int fd, std::size_t want, int timeout_ms = 1000) {
  std::vector<std::uint8_t> out;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (out.size() < want && std::chrono::steady_clock::now() < deadline) {
    std::uint8_t buf[512];
    const ssize_t n = ::read(fd, buf, sizeof(buf));
    if (n > 0) {
      out.insert(out.end(), buf, buf + n);
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  return out;
}
}  // namespace

TEST(VisionLink, RelaysBothWaysAndTapsFrames) {
  const std::string dir = TempDir("vision");
  devices::Transports transports(dir);
  sim::FakeSim sim;
  devices::VisionLink link;
  devices::DeviceSpec spec;
  spec.name = "vision";
  spec.type = "vision_link";
  spec.transport = YAML::Load("{mcb: vision_mcb, otto: vision_otto}");
  spec.bind = YAML::Node(YAML::NodeType::Map);
  spec.params = YAML::Load("{log: vision.jsonl}");
  link.Configure(spec);
  link.Bind(sim, transports);
  std::vector<devices::VisionLink::Message> tapped;
  link.AddListener([&](const devices::VisionLink::Message& m) { tapped.push_back(m); });

  const int mcb = hwemu_serial_open(link.McbPath().c_str());
  const int otto = hwemu_serial_open(link.OttoPath().c_str());
  ASSERT_GE(mcb, 0);
  ASSERT_GE(otto, 0);

  // The MCB sends odometry; otto receives exactly those bytes.
  protocol::vision::Odometry odom;
  odom.timestamp_us = 42;
  odom.turrets.push_back({});
  const auto frame = protocol::EncodeDjiFrame(0x01, protocol::vision::Encode(odom), 3);
  ASSERT_EQ(::write(mcb, frame.data(), frame.size()), static_cast<ssize_t>(frame.size()));
  // And otto sends an aim message the other way.
  std::vector<protocol::vision::TurretAim> aim(1);
  aim[0].timestamp_us = 42;
  const auto reply = protocol::EncodeDjiFrame(0x02, protocol::vision::Encode(std::span<const protocol::vision::TurretAim>(aim)));
  ASSERT_EQ(::write(otto, reply.data(), reply.size()), static_cast<ssize_t>(reply.size()));
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  link.Tick({1, 1000, 0.001, 0.001});

  EXPECT_EQ(ReadAll(otto, frame.size()), frame);
  EXPECT_EQ(ReadAll(mcb, reply.size()), reply);
  ASSERT_EQ(tapped.size(), 2U);
  EXPECT_TRUE(tapped[0].from_mcb);
  EXPECT_EQ(tapped[0].frame.cmd, 0x01);
  EXPECT_FALSE(tapped[1].from_mcb);
  EXPECT_EQ(tapped[1].frame.cmd, 0x02);
  ::close(mcb);
  ::close(otto);
  std::ifstream log(dir + "/vision.jsonl");
  std::string line;
  std::getline(log, line);
  EXPECT_NE(line.find("\"dir\":\"mcb->otto\""), std::string::npos);
  EXPECT_NE(line.find("\"type\":\"odometry\""), std::string::npos);
}

TEST(FrameShm, AcrossProcesses) {
  transport::FrameShmWriter writer("hwemu_test_ring", 3, 16, 1024);
  const pid_t child = ::fork();
  if (child == 0) {
    // The reader, in another process: waits for frame 5 and checks it whole.
    transport::FrameShmReader reader("hwemu_test_ring");
    transport::FrameShmReader::Frame f;
    for (int i = 0; i < 2000; ++i) {
      if (reader.ReadLatest(&f) && f.sequence >= 5) {
        const bool ok = f.payload.size() == 1000 && f.payload[999] == static_cast<std::uint8_t>(f.sequence) &&
                        f.meta.size() == 4;
        ::_exit(ok ? 0 : 2);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ::_exit(1);
  }
  for (std::uint64_t s = 1; s <= 50; ++s) {
    std::vector<std::uint8_t> payload(1000, static_cast<std::uint8_t>(s));
    const std::uint8_t meta[4] = {1, 2, 3, 4};
    writer.Write(meta, {payload}, static_cast<double>(s));
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  int status = 0;
  ::waitpid(child, &status, 0);
  EXPECT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0);
}

TEST(FrameShm, AnOldWriterLeavesANewWritersRingInPlace) {
  // A hub still shutting down when the next run's hub has made its rings
  // (a rerun right after Ctrl-C) must not unlink them on its way out.
  auto old_writer = std::make_unique<transport::FrameShmWriter>("hwemu_test_handover", 2, 16, 64);
  transport::FrameShmWriter new_writer("hwemu_test_handover", 2, 16, 64);
  old_writer.reset();
  EXPECT_TRUE(std::filesystem::exists("/dev/shm/hwemu_test_handover"));
  transport::FrameShmReader reader("hwemu_test_handover");  // throws if it's gone
}

TEST(FrameShm, AWriterUnlinksItsOwnRing) {
  { transport::FrameShmWriter writer("hwemu_test_own", 2, 16, 64); }
  EXPECT_FALSE(std::filesystem::exists("/dev/shm/hwemu_test_own"));
}

TEST(SimClock, PublishesMonotonicMicros) {
  clock::ClockOptions options;
  options.mode = clock::ClockMode::kLockstep;  // no participants: runs flat out
  options.dt = 0.001;
  options.shm_name = "hwemu_test_clock";
  clock::SimClock clk(options);
  ::setenv("HWEMU_CLOCK_SHM", "hwemu_test_clock", 1);
  for (int i = 0; i < 2500; ++i) {
    ASSERT_TRUE(clk.WaitForNextTick());
    clk.Advance();
  }
  EXPECT_EQ(clk.Micros(), 2500000U);
  EXPECT_EQ(hwemu_clock_micros(), 2500000U);
}

TEST(SimClock, RealtimeIsPacedToTheWall) {
  clock::ClockOptions options;
  options.dt = 0.002;
  options.shm_name = "";
  clock::SimClock clk(options);
  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < 50; ++i) {
    clk.WaitForNextTick();
    clk.Advance();
  }
  const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  EXPECT_GT(wall, 0.09);
  EXPECT_LT(wall, 0.5);
}

TEST(SimClock, LockstepWaitsForEveryParticipantInOrder) {
  const std::string dir = TempDir("lockstep");
  clock::ClockOptions options;
  options.mode = clock::ClockMode::kLockstep;
  options.dt = 0.001;
  options.lockstep_socket = dir + "/clock.sock";
  options.order = clock::LockstepOrder::kSequential;
  options.expected = {"mcb", "otto"};
  options.shm_name = "";
  clock::SimClock clk(options);

  std::mutex m;
  std::vector<std::string> order;
  auto participant = [&](const char* name) {
    const int h = hwemu_lockstep_join(options.lockstep_socket.c_str(), name);
    ASSERT_GE(h, 0);
    for (int i = 0; i < 5; ++i) {
      std::uint64_t tick = 0, micros = 0;
      ASSERT_EQ(hwemu_lockstep_wait(h, &tick, &micros), 0);
      EXPECT_EQ(micros, tick * 1000);
      {
        std::lock_guard<std::mutex> l(m);
        order.push_back(std::string(name) + std::to_string(tick));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      hwemu_lockstep_ack(h, tick);
    }
    hwemu_lockstep_leave(h);
  };
  // otto joins first, but `expected` puts the MCB first in each tick.
  std::thread otto(participant, "otto");
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  std::thread mcb(participant, "mcb");
  for (int i = 0; i < 5; ++i) {
    ASSERT_TRUE(clk.WaitForNextTick());
    clk.Advance();
  }
  ASSERT_TRUE(clk.WaitForNextTick());  // collects tick 5's acks
  otto.join();
  mcb.join();
  ASSERT_EQ(order.size(), 10U);
  for (int t = 1; t <= 5; ++t) {
    EXPECT_EQ(order[(t - 1) * 2], "mcb" + std::to_string(t));
    EXPECT_EQ(order[(t - 1) * 2 + 1], "otto" + std::to_string(t));
  }
}
