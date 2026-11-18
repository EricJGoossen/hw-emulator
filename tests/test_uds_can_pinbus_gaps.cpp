// The UDS CAN bus against the client library, the pinbus's bus-routed SPI,
// PWM and ADC, the gap report, and device plugins.
#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <set>
#include <thread>

#include <unistd.h>

#include "hwemu/devices/device.hpp"
#include "hwemu/gaps.hpp"
#include "hwemu/transport/can_bus.hpp"
#include "hwemu/transport/pinbus.hpp"
#include "hwemu_client.h"

using namespace hwemu;

namespace {

std::string TempDir(const std::string& name) {
  auto dir = std::filesystem::temp_directory_path() / ("hwemu_test_" + name + "_" + std::to_string(::getpid()));
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  return dir.string();
}

template <typename F>
bool Eventually(F f, int ms = 1000) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (f()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return f();
}

std::string Slurp(const std::string& path) {
  std::ifstream f(path);
  std::stringstream s;
  s << f.rdbuf();
  return s.str();
}

}  // namespace

TEST(UdsCan, HubAndClientsShareTheBus) {
  const std::string dir = TempDir("udscan");
  transport::CanBusSet set;
  set.SetSocketDir(dir);
  transport::ICanBus* bus = set.Open("can1");
  ASSERT_TRUE(std::filesystem::exists(dir + "/can1.sock"));

  const int a = hwemu_can_open((dir + "/can1.sock").c_str());
  const int b = hwemu_can_open((dir + "/can1.sock").c_str());
  ASSERT_GE(a, 0);
  ASSERT_GE(b, 0);

  // A client's frame reaches the hub, and the other client, unchanged.
  const std::uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  ASSERT_EQ(hwemu_can_send(a, 0x200, 0, data, 8), 0);
  protocol::CanFrame f;
  ASSERT_TRUE(Eventually([&] { return bus->Receive(&f); }));
  EXPECT_EQ(f.id, 0x200U);
  EXPECT_FALSE(f.extended);
  EXPECT_EQ(f.len, 8);
  EXPECT_EQ(f.data[7], 8);

  std::uint32_t id = 0;
  int ext = 0;
  std::uint8_t rx[8] = {};
  std::uint8_t len = 0;
  ASSERT_TRUE(Eventually([&] { return hwemu_can_recv(b, &id, &ext, rx, &len) == 1; }));
  EXPECT_EQ(id, 0x200U);

  // The hub's frame reaches every client, extended ids intact.
  protocol::CanFrame out;
  out.id = 0x2901;
  out.extended = true;
  out.len = 2;
  out.data[0] = 0xAB;
  // Let the hub accept both clients first.
  ASSERT_TRUE(Eventually([&] { return dynamic_cast<transport::UdsCanBus*>(bus)->Send(out) &&
                                      dynamic_cast<transport::UdsCanBus*>(bus)->Clients() == 2; }));
  for (int fd : {a, b}) {
    ASSERT_TRUE(Eventually([&] {
      return hwemu_can_recv(fd, &id, &ext, rx, &len) == 1 && id == 0x2901U;
    })) << "client " << fd;
    EXPECT_EQ(ext, 1);
    EXPECT_EQ(len, 2);
    EXPECT_EQ(rx[0], 0xAB);
  }
  hwemu_can_close(a);
  hwemu_can_close(b);
}

TEST(UdsCan, AnIdNothingConsumesIsAGap) {
  gaps::Reset();
  auto [ours, theirs] = transport::LoopbackCanBus::Pair("can9");
  transport::CanRouter router;
  router.Subscribe(ours.get(), 0x201, false, [](const protocol::CanFrame&) {});
  protocol::CanFrame f;
  f.id = 0x333;
  f.len = 1;
  theirs->Send(f);
  theirs->Send(f);  // reported once, not twice
  router.Poll();
  const auto all = gaps::All();
  ASSERT_EQ(all.size(), 1U);
  EXPECT_EQ(all[0].source, "can");
  EXPECT_NE(all[0].message.find("0x333"), std::string::npos);
}

TEST(PinBus, SpiGoesToWhicheverChipSelectIsLow) {
  gaps::Reset();
  const std::string dir = TempDir("pinbus_cs");
  transport::PinBusServer server(dir + "/pinbus.sock");
  server.RegisterSpi("imu_a", [](std::span<const std::uint8_t> tx) {
    return std::vector<std::uint8_t>(tx.size(), 0xA0);
  });
  server.RegisterSpi("imu_b", [](std::span<const std::uint8_t> tx) {
    return std::vector<std::uint8_t>(tx.size(), 0xB0);
  });
  server.RegisterSpiOnBus("imu_a", "spi5", "CsA");
  server.RegisterSpiOnBus("imu_b", "spi5", "CsB");

  const int h = hwemu_pinbus_connect(server.Path().c_str(), "test");
  ASSERT_GE(h, 0);
  std::uint8_t tx[2] = {0x80, 0}, rx[2] = {};
  hwemu_gpio_write(h, "CsA", 1);
  hwemu_gpio_write(h, "CsB", 1);
  EXPECT_EQ(hwemu_spi_transfer(h, "bus:spi5", tx, rx, 2), -1);  // none selected
  EXPECT_EQ(rx[1], 0xFF);

  hwemu_gpio_write(h, "CsB", 0);
  ASSERT_EQ(hwemu_spi_transfer(h, "bus:spi5", tx, rx, 2), 0);
  EXPECT_EQ(rx[1], 0xB0);
  hwemu_gpio_write(h, "CsB", 1);
  hwemu_gpio_write(h, "CsA", 0);
  ASSERT_EQ(hwemu_spi_transfer(h, "bus:spi5", tx, rx, 2), 0);
  EXPECT_EQ(rx[1], 0xA0);

  // Chip-select writes are modeled, not gaps; the empty selection was.
  bool none_gap = false, cs_gap = false;
  for (const auto& g : gaps::All()) {
    none_gap = none_gap || g.key == "spi:spi5:none";
    cs_gap = cs_gap || g.key.rfind("gpio-out:Cs", 0) == 0;
  }
  EXPECT_TRUE(none_gap);
  EXPECT_FALSE(cs_gap);
  hwemu_pinbus_close(h);
}

TEST(PinBus, PwmAndAdcAndTheirGaps) {
  gaps::Reset();
  const std::string dir = TempDir("pinbus_pwm");
  transport::PinBusServer server(dir + "/pinbus.sock");
  float seen = -1;
  server.OnPwm("FanX", [&](const std::string&, float duty, std::uint32_t) { seen = duty; });
  server.AcceptOutput("BuzzerStub");
  server.SetAnalog("S", 2048);

  const int h = hwemu_pinbus_connect(server.Path().c_str(), "test");
  ASSERT_GE(h, 0);
  ASSERT_EQ(hwemu_pwm_write(h, "FanX", 0.25F, 25000), 0);
  EXPECT_FLOAT_EQ(seen, 0.25F);
  EXPECT_EQ(server.Pwm("FanX").second, 25000U);
  ASSERT_EQ(hwemu_pwm_write(h, "BuzzerStub", 0.5F, 2000), 0);  // a stub: accepted quietly
  ASSERT_EQ(hwemu_pwm_write(h, "Unknown", 0.5F, 50), 0);       // a gap
  EXPECT_EQ(hwemu_adc_read(h, "S"), 2048);
  EXPECT_EQ(hwemu_adc_read(h, "T"), 0);  // nothing drives it: 0, and a gap

  std::set<std::string> keys;
  for (const auto& g : gaps::All()) keys.insert(g.key);
  EXPECT_TRUE(keys.count("pwm:Unknown"));
  EXPECT_TRUE(keys.count("adc:T"));
  EXPECT_FALSE(keys.count("pwm:FanX"));
  EXPECT_FALSE(keys.count("pwm:BuzzerStub"));
  hwemu_pinbus_close(h);
}

TEST(Gaps, WrittenOncePerKeyToTheProcessFile) {
  gaps::Reset();
  const std::string dir = TempDir("gaps");
  gaps::Configure(dir, "hubtest");
  gaps::SetTime(1.5);
  gaps::Report("dev", "k1", "first \"quoted\"");
  gaps::Report("dev", "k1", "again");
  gaps::Report("dev", "k2", "second");
  const std::string text = Slurp(dir + "/gaps/hubtest.jsonl");
  EXPECT_EQ(std::count(text.begin(), text.end(), '\n'), 2);
  EXPECT_NE(text.find("\"key\":\"k1\""), std::string::npos);
  EXPECT_NE(text.find("first \\\"quoted\\\""), std::string::npos);
  EXPECT_NE(text.find("\"time\":1.5"), std::string::npos);

  // The client library writes the same format, to its own process's file.
  ::setenv("HWEMU_RUNTIME_DIR", dir.c_str(), 1);
  ::setenv("HWEMU_PROCESS", "mcbtest", 1);
  hwemu_report_gap("uart", "remote", "no DR16");
  hwemu_report_gap("uart", "remote", "no DR16");
  const std::string client = Slurp(dir + "/gaps/mcbtest.jsonl");
  EXPECT_EQ(std::count(client.begin(), client.end(), '\n'), 1);
  EXPECT_NE(client.find("\"source\":\"uart\""), std::string::npos);
  gaps::Configure(TempDir("gaps_done"), "hub");
}
