// The CAN codecs and IMU register maps, against taproot's and aruwsrc's
// decode logic (dji_motor.cpp, dji_motor_encoder.cpp, can_encoder.cpp,
// lamprey_encoder.hpp, turret_mcb_can_comm.cpp, tmotor_ak80_9*.cpp,
// mpu6500.cpp, ism330.cpp) restated here.
#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include "hwemu/protocol/can_encoder.hpp"
#include "hwemu/protocol/dji_motor_can.hpp"
#include "hwemu/protocol/spi_imu.hpp"
#include "hwemu/protocol/tmotor_can.hpp"
#include "hwemu/protocol/turret_mcb_can.hpp"

using namespace hwemu::protocol;

TEST(DjiMotor, FeedbackDecodesAsTaprootDoes) {
  const CanFrame f = dji_motor::EncodeFeedback(0x205, {8000, -1234, 512, 41});
  EXPECT_EQ(f.id, 0x205U);
  EXPECT_FALSE(f.extended);
  // taproot: encoder = data[0] << 8 | data[1]; rpm = int16(data[2] << 8 | data[3]);
  //          torque = int16(data[4] << 8 | data[5]); temperature = int8(data[6]).
  EXPECT_EQ(static_cast<std::uint16_t>(f.data[0] << 8 | f.data[1]), 8000);
  EXPECT_EQ(static_cast<std::int16_t>(f.data[2] << 8 | f.data[3]), -1234);
  EXPECT_EQ(static_cast<std::int16_t>(f.data[4] << 8 | f.data[5]), 512);
  EXPECT_EQ(static_cast<std::int8_t>(f.data[6]), 41);
  EXPECT_EQ(dji_motor::DecodeFeedback(f), (dji_motor::Feedback{8000, -1234, 512, 41}));
}

TEST(DjiMotor, CommandSlotsMatchTxHandler) {
  using dji_motor::CommandSlot;
  EXPECT_EQ(*dji_motor::CommandSlotFor(0x201, false), (CommandSlot{0x200, 0}));
  EXPECT_EQ(*dji_motor::CommandSlotFor(0x204, true), (CommandSlot{0x200, 3}));  // low ids ignore current
  EXPECT_EQ(*dji_motor::CommandSlotFor(0x206, false), (CommandSlot{0x1FF, 1}));
  EXPECT_EQ(*dji_motor::CommandSlotFor(0x206, true), (CommandSlot{0x1FE, 1}));
  EXPECT_EQ(*dji_motor::CommandSlotFor(0x209, false), (CommandSlot{0x2FF, 0}));
  EXPECT_FALSE(dji_motor::CommandSlotFor(0x200, false));

  // taproot: data[2 * id] = out >> 8; data[2 * id + 1] = out & 0xFF, id = (n - 1) % 4.
  const CanFrame f = dji_motor::EncodeCommand(0x1FF, {100, -200, 300, -16384});
  EXPECT_EQ(f.data[2], static_cast<std::uint8_t>((-200 >> 8) & 0xFF));
  EXPECT_EQ(f.data[3], static_cast<std::uint8_t>(-200 & 0xFF));
  EXPECT_EQ(*dji_motor::DecodeCommand(f, {0x1FF, 3}), -16384);
  EXPECT_FALSE(dji_motor::DecodeCommand(f, {0x200, 3}));
}

TEST(CanEncoder, LittleEndianCountsAndGauss) {
  const CanFrame f = can_encoder::Encode(0x1E2, {4095, 700});
  EXPECT_EQ((f.data[1] << 8) | f.data[0], 4095);
  EXPECT_EQ((f.data[3] << 8) | f.data[2], 700);
  EXPECT_EQ(can_encoder::AngleToCounts(std::numbers::pi), 2048);
  EXPECT_EQ(can_encoder::AngleToCounts(-std::numbers::pi / 2), 3072);
  EXPECT_EQ(can_encoder::AngleToCounts(4 * std::numbers::pi + 0.0001), 0);
}

TEST(CanEncoder, LampreyCentidegrees) {
  // aruwsrc: angleRaw = toRadian(raw / 100.0)
  const std::uint16_t raw = can_encoder::AngleToCentidegrees(1.0);
  EXPECT_NEAR(raw / 100.0 * std::numbers::pi / 180.0, 1.0, 1e-3);
  const CanFrame f = can_encoder::EncodeLamprey(0x1E0, raw);
  EXPECT_EQ(*can_encoder::DecodeLamprey(f), raw);
  EXPECT_EQ(can_encoder::AngleToCentidegrees(-0.5 * std::numbers::pi), 27000);
}

TEST(TurretMcb, AxisMessageDecodesAsTurretMcbCanComm) {
  using namespace turret_mcb;
  const double roll = 1.2, rate = -3.0, accel = 9.81;
  const CanFrame f = EncodeAxis(kXAxisId, {AngleToFixed(roll), RateToRaw(rate), AccelerationToRaw(accel), 42});
  EXPECT_EQ(f.len, 7);
  const auto m = DecodeAxis(f);
  ASSERT_TRUE(m);
  // aruwsrc: roll = angleFixedPoint * 2pi / UINT16_MAX; rate / LSB_PER_RAD_PER_S;
  //          accel = linearAcceleration * 0.01.
  EXPECT_NEAR(m->angle * (2 * std::numbers::pi / 65535.0), roll, 1e-4);
  EXPECT_NEAR(m->rate / (16.384 * 180.0 / std::numbers::pi), rate, 2e-3);
  EXPECT_NEAR(m->acceleration * 0.01, accel, 0.01);
  EXPECT_EQ(m->seq, 42);
  // Wrapped into (-pi, pi] first.
  EXPECT_NEAR(FixedToAngle(AngleToFixed(3 * std::numbers::pi / 2)), -std::numbers::pi / 2, 1e-4);
}

TEST(TurretMcb, StatusAndHandshakes) {
  using namespace turret_mcb;
  const CanFrame s = EncodeStatus({kKickerLimitSwitch, ImuState::kCalibrating, 4012});
  EXPECT_EQ(s.id, kStatusId);
  EXPECT_EQ(s.data[0] & 0b1, 1);
  EXPECT_EQ(s.data[1], 2);
  EXPECT_EQ(static_cast<std::int16_t>(s.data[2] | (s.data[3] << 8)), 4012);
  EXPECT_EQ(DecodeStatus(s)->imu_state, ImuState::kCalibrating);

  EXPECT_EQ(*DecodeCommand(EncodeCommand(kRecalibrateImu | kLaserOn)), kRecalibrateImu | kLaserOn);
  EXPECT_EQ(*DecodeSyncResponse(EncodeSyncResponse(0xDEADBEEF)), 0xDEADBEEFU);
  EXPECT_EQ(*DecodeCalibrationSamples(EncodeCalibrationSamples(1500)), 1500);

  MountingMessage m;
  m.imu = RemoteImuType::kIsm330;
  m.part = TransformPart::kTranslation;
  m.raw = {static_cast<std::uint16_t>(static_cast<std::int16_t>(-16383)), 16383, 0};
  const auto values = DecodeMounting(EncodeMounting(m))->Values();
  EXPECT_NEAR(values[0], -1.0, 1e-3);
  EXPECT_NEAR(values[1], 1.0, 1e-3);
}

TEST(Tmotor, FeedbackAndCommand) {
  using namespace tmotor;
  const CanFrame f = EncodeFeedback(3, {-900, 1200, -250, 35, 0});
  EXPECT_TRUE(f.extended);
  EXPECT_EQ(f.id, 0x2903U);
  // aruwsrc: int16(data[0] << 8 | data[1]) etc.
  EXPECT_EQ(static_cast<std::int16_t>(f.data[0] << 8 | f.data[1]), -900);
  EXPECT_EQ(*DecodeFeedback(f), (Feedback{-900, 1200, -250, 35, 0}));

  const CanFrame c = EncodeCurrentCommand(3, -12345);
  EXPECT_EQ(c.id, 0x103U);
  EXPECT_EQ(*DecodeCurrentCommand(c, 3), -12345);
  EXPECT_FALSE(DecodeCurrentCommand(c, 4));
}

TEST(SpiImu, Mpu6500BurstReadMatchesTaproot) {
  spi_imu::Mpu6500 imu;
  // taproot's init: WHO_AM_I, then FS +/-2000 dps and +/-8 g.
  EXPECT_EQ(imu.Transfer(std::vector<std::uint8_t>{0x75 | 0x80, 0})[1], 0x70);
  imu.Transfer(std::vector<std::uint8_t>{0x1B, 0x18});
  imu.Transfer(std::vector<std::uint8_t>{0x1C, 0x10});
  imu.SetSample({{0.5, -0.25, 1.0}, {0.0, 0.0, spi_imu::kGravity}, 30.0});
  std::vector<std::uint8_t> tx(15, 0);
  tx[0] = 0x3B | 0x80;
  const auto rx = imu.Transfer(tx);
  auto be = [&](int i) { return static_cast<std::int16_t>(rx[1 + i] << 8 | rx[2 + i]); };
  EXPECT_NEAR(be(4) / 4096.0, 1.0, 1e-3);                              // az, g
  EXPECT_NEAR(21.0 + be(6) / 333.87, 30.0, 0.01);                       // temperature
  EXPECT_NEAR(be(8) / 16.4 * std::numbers::pi / 180.0, 0.5, 1e-3);      // gx
  EXPECT_NEAR(be(10) / 16.4 * std::numbers::pi / 180.0, -0.25, 1e-3);   // gy
}

TEST(SpiImu, Ism330LittleEndianAndRanges) {
  spi_imu::Ism330dhcx imu;
  EXPECT_EQ(imu.Transfer(std::vector<std::uint8_t>{0x0F | 0x80, 0})[1], 0x6B);
  imu.Transfer(std::vector<std::uint8_t>{0x10, 0x0C});  // 8 g
  imu.Transfer(std::vector<std::uint8_t>{0x11, 0x0C});  // 2000 dps
  imu.SetSample({{1.0, 0, 0}, {0, 0, -spi_imu::kGravity}, 25.0});
  std::vector<std::uint8_t> tx(15, 0);
  tx[0] = 0x20 | 0x80;
  const auto rx = imu.Transfer(tx);
  auto le = [&](int i) { return static_cast<std::int16_t>(rx[1 + i] | (rx[2 + i] << 8)); };
  EXPECT_EQ(le(0), 0);                                                        // 25 degC
  EXPECT_NEAR(le(2) * 70.0 / 1000.0 * std::numbers::pi / 180.0, 1.0, 2e-3);  // gx
  EXPECT_NEAR(le(12) * 0.244 / 1000.0, -1.0, 1e-3);                          // az, g
}

TEST(SpiImu, Bmi088AccelHasDummyByte) {
  spi_imu::Bmi088Accel accel;
  const auto id = accel.Transfer(std::vector<std::uint8_t>{0x80, 0, 0});
  EXPECT_EQ(id[2], 0x1E);  // byte 1 is the dummy
  spi_imu::Bmi088Gyro gyro;
  EXPECT_EQ(gyro.Transfer(std::vector<std::uint8_t>{0x80, 0})[1], 0x0F);
  gyro.SetSample({{0, 0, 2.0}, {}, 25.0});
  const auto rx = gyro.Transfer(std::vector<std::uint8_t>{0x02 | 0x80, 0, 0, 0, 0, 0, 0});
  EXPECT_NEAR(static_cast<std::int16_t>(rx[5] | (rx[6] << 8)) / 16.384 * std::numbers::pi / 180.0, 2.0, 1e-3);
}
