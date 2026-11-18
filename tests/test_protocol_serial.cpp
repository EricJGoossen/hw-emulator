// CRCs, DJI serial framing, the vision messages and LD19 packets -- against
// project-otto's own encodings (golden/).
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <string_view>

#include "golden.hpp"
#include "hwemu/protocol/crc.hpp"
#include "hwemu/protocol/dji_serial.hpp"
#include "hwemu/protocol/ld19.hpp"
#include "hwemu/protocol/vision.hpp"

using namespace hwemu::protocol;

namespace {
std::vector<std::uint8_t> Bytes(std::string_view s) { return {s.begin(), s.end()}; }
}  // namespace

TEST(Crc, CheckValues) {
  const auto check = Bytes("123456789");
  EXPECT_EQ(Crc8Dji(check), 0x0B);
  EXPECT_EQ(Crc16Dji(check), 0x6F91);
  EXPECT_EQ(Crc8Dji(check), Golden("crc8_check")[0]);
  const auto g16 = Golden("crc16_check");
  EXPECT_EQ(Crc16Dji(check), g16[0] | (g16[1] << 8));
}

TEST(Crc, Ld19TableMatchesOtto) {
  // otto's LD19 table: [1] = 0x4D, [2] = 0x9A, [255] = 0xA8.
  const std::uint8_t one[] = {0x01};
  const std::uint8_t two[] = {0x02};
  const std::uint8_t last[] = {0xFF};
  EXPECT_EQ(Crc8Ld19(one), 0x4D);
  EXPECT_EQ(Crc8Ld19(two), 0x9A);
  EXPECT_EQ(Crc8Ld19(last), 0xA8);
}

TEST(DjiSerial, EncodesLikeOttoTransceiver) {
  EXPECT_EQ(EncodeDjiFrame(0x08, {}, 0), Golden("frame_empty_reboot"));
  const auto golden = Golden("frame_odometry");
  const std::vector<std::uint8_t> data(golden.begin() + 7, golden.end() - 2);
  EXPECT_EQ(EncodeDjiFrame(0x01, data, 7), golden);
}

TEST(DjiSerial, ParsesGoldenFramesAndResyncs) {
  DjiFrameParser parser;
  std::vector<std::uint8_t> stream = {0x00, 0xA5, 0x13};  // junk, and a false SOF
  for (const char* name : {"frame_odometry", "frame_turret_aim", "frame_auto_nav", "frame_health"}) {
    auto g = Golden(name);
    stream.insert(stream.end(), g.begin(), g.end());
  }
  // A frame with a broken CRC16 between two good ones costs only itself.
  auto bad = Golden("frame_empty_reboot");
  bad.back() ^= 0xFF;
  const auto first = Golden("frame_odometry").size() + 3;
  stream.insert(stream.begin() + static_cast<std::ptrdiff_t>(first), bad.begin(), bad.end());

  // Fed a byte at a time, as a UART delivers it.
  std::vector<DjiFrame> frames;
  for (std::uint8_t b : stream) {
    parser.Feed({&b, 1});
    while (auto f = parser.Next()) frames.push_back(*f);
  }
  ASSERT_EQ(frames.size(), 4U);
  EXPECT_EQ(frames[0].cmd, 0x01);
  EXPECT_EQ(frames[0].seq, 7);
  EXPECT_EQ(frames[1].cmd, 0x02);
  EXPECT_EQ(frames[2].cmd, 0x0D);
  EXPECT_EQ(frames[3].cmd, 0x0C);
  EXPECT_GT(parser.GetStats().discarded_bytes, 0U);
  EXPECT_EQ(parser.GetStats().frame_crc_errors, 1U);
}

TEST(DjiSerial, RoundTrip) {
  std::vector<std::uint8_t> data(300);
  for (std::size_t i = 0; i < data.size(); ++i) data[i] = static_cast<std::uint8_t>(i * 7);
  DjiFrameParser parser;
  parser.Feed(EncodeDjiFrame(0x1234, data, 200));
  auto f = parser.Next();
  ASSERT_TRUE(f);
  EXPECT_EQ(f->cmd, 0x1234);
  EXPECT_EQ(f->seq, 200);
  EXPECT_EQ(f->data, data);
}

namespace {
std::vector<std::uint8_t> Body(const std::string& golden) {
  const auto g = Golden(golden);
  return {g.begin() + 7, g.end() - 2};
}
}  // namespace

TEST(Vision, OdometryMatchesOtto) {
  const auto data = Body("frame_odometry");
  auto m = vision::DecodeOdometry(data);
  ASSERT_TRUE(m);
  EXPECT_EQ(m->timestamp_us, 123456U);
  EXPECT_FLOAT_EQ(m->chassis.x, 1.0F);
  EXPECT_FLOAT_EQ(m->chassis.yaw, 3.0F);
  ASSERT_EQ(m->turrets.size(), 1U);
  EXPECT_FLOAT_EQ(m->turrets[0].yaw, -1.5F);
  EXPECT_EQ(vision::Encode(*m), data);
}

TEST(Vision, TurretAimMatchesOtto) {
  const auto data = Body("frame_turret_aim");
  auto m = vision::DecodeTurretAim(data);
  ASSERT_TRUE(m);
  ASSERT_EQ(m->size(), 2U);
  ASSERT_TRUE((*m)[0].target);
  EXPECT_EQ((*m)[0].timestamp_us, 99U);
  EXPECT_EQ((*m)[0].target->fire_rate, 2);
  EXPECT_FLOAT_EQ((*m)[0].target->velocity[2], 6.0F);
  EXPECT_FLOAT_EQ((*m)[0].target->theta, 0.5F);
  EXPECT_EQ((*m)[0].target->icon, 3);
  EXPECT_FALSE((*m)[1].target);
  EXPECT_EQ((*m)[1].timestamp_us, 100U);
  EXPECT_EQ(vision::Encode(std::span<const vision::TurretAim>(*m)), data);
}

TEST(Vision, AutoNavAndTransformsAndHealthMatchOtto) {
  auto nav = vision::DecodeAutoNavSetpoint(Body("frame_auto_nav"));
  ASSERT_TRUE(nav);
  EXPECT_EQ(nav->sequence, 5U);
  ASSERT_EQ(nav->points.size(), 2U);
  EXPECT_FLOAT_EQ(nav->points[1].second, 4.0F);
  EXPECT_EQ(vision::Encode(*nav), Body("frame_auto_nav"));

  auto tf = vision::DecodeTransformUpdate(Body("frame_world_to_turret"));
  ASSERT_TRUE(tf);
  EXPECT_FALSE(tf->tag);
  EXPECT_EQ(tf->timestamp_us, 555U);
  EXPECT_EQ(tf->turret_id, 1);
  EXPECT_EQ(vision::Encode(*tf), Body("frame_world_to_turret"));

  auto hp = vision::DecodeHealth(Body("frame_health"));
  ASSERT_TRUE(hp);
  EXPECT_EQ((*hp)[0], 100);
  EXPECT_EQ((*hp)[13], 1400);
}

TEST(Vision, DescribeIsJson) {
  const std::string s = vision::Describe(0x01, Body("frame_odometry"));
  EXPECT_NE(s.find("\"type\":\"odometry\""), std::string::npos);
  EXPECT_NE(s.find("\"timestamp_us\":123456"), std::string::npos);
  const std::string bad = vision::Describe(0x0C, std::vector<std::uint8_t>(3));
  EXPECT_NE(bad.find("error"), std::string::npos);
  EXPECT_EQ(vision::DirectionOf(0x02), vision::Direction::kJetsonToMcb);
  EXPECT_STREQ(vision::Name(0x0D), "auto_nav_setpoint");
}

TEST(Ld19, MatchesOttoPacket) {
  const auto golden = Golden("ld19_packet");
  auto p = ld19::Decode(golden);
  ASSERT_TRUE(p);
  EXPECT_EQ(p->speed_dps, 3600);
  EXPECT_EQ(p->start_angle_cdeg, 1000);
  EXPECT_EQ(p->points[11].distance_mm, 1011);
  EXPECT_EQ(p->points[0].intensity, 200);
  EXPECT_EQ(p->end_angle_cdeg, 2100);
  EXPECT_EQ(p->timestamp_ms, 1234);
  const auto encoded = ld19::Encode(*p);
  EXPECT_EQ(std::vector<std::uint8_t>(encoded.begin(), encoded.end()), golden);
}

TEST(Ld19, ScanIsClockwiseAndMissesAreZero) {
  // 24 beams counterclockwise from +x; beam 3 sees nothing.
  std::vector<float> ranges(24, 2.0F), angles(24);
  for (int i = 0; i < 24; ++i) angles[i] = static_cast<float>(i * 2.0 * std::numbers::pi / 24.0);
  ranges[3] = std::numeric_limits<float>::infinity();
  auto packets = ld19::PacketsForScan(ranges, angles, 10.0, 45000);
  ASSERT_EQ(packets.size(), 2U);
  EXPECT_EQ(packets[0].speed_dps, 3600);
  EXPECT_EQ(packets[0].timestamp_ms, 15000);  // 45000 wraps at 30000
  EXPECT_EQ(packets[0].start_angle_cdeg, 0);
  // Beam 3 at 45 deg counterclockwise is 315 deg clockwise: the second packet.
  bool found_miss = false;
  for (const auto& pt : packets[1].points) found_miss |= pt.distance_mm == 0;
  EXPECT_TRUE(found_miss);
  EXPECT_EQ(packets[1].points[0].distance_mm, 2000);
  EXPECT_LT(packets[0].start_angle_cdeg, packets[0].end_angle_cdeg);
}
