// Backend parity: the turret testbed (examples/turret_testbed) driven the
// same way on Achilles and on MuJoCo, read through the same sim kinds. The
// two engines integrate differently and model their sensors' error
// differently, so they agree to a tolerance, not exactly -- what this pins
// is that the models, the units and the sign conventions match: the same
// voltage turns the same joint the same way at about the same rate, the
// gyro and attitude see it about the same axis, gravity reads the same way
// up, and the limit switch closes at the same angle.
#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "hwemu/config/config.hpp"
#include "hwemu/sim/sim.hpp"

#if defined(HWEMU_WITH_ACHILLES) && defined(HWEMU_WITH_MUJOCO)

namespace {

using hwemu::sim::Backend;
using hwemu::sim::ISim;
using hwemu::sim::MotorCommand;

const std::string kDir = std::string(HWEMU_EXAMPLES_DIR) + "/turret_testbed/";

std::unique_ptr<ISim> Make(Backend backend) {
  auto config = hwemu::config::LoadHubConfig(kDir + "sim.yaml", kDir + "devices.yaml");
  config.sim.backend = backend;
  config.sim.display = hwemu::sim::DisplayMode::kHeadless;
  return hwemu::sim::CreateSim(config.sim);
}

struct Rig {
  std::unique_ptr<ISim> sim;
  explicit Rig(Backend b) : sim(Make(b)) {}
  void For(double seconds, double dt = 0.001) {
    const int n = static_cast<int>(std::lround(seconds / dt));
    for (int i = 0; i < n; ++i) sim->Step(dt);
  }
};

class Parity : public ::testing::Test {
 protected:
  Rig achilles{Backend::kAchilles};
  Rig mujoco{Backend::kMujoco};
  void Both(const std::function<void(Rig&)>& f) {
    f(achilles);
    f(mujoco);
  }
};

TEST_F(Parity, AtRestBothReadGravityUpAndNoMotion) {
  Both([](Rig& r) { r.For(0.1); });
  for (Rig* r : {&achilles, &mujoco}) {
    SCOPED_TRACE(std::string(hwemu::sim::Name(r->sim->GetBackend())));
    // Averaged: one sample's noise (0.01 m/s^2/sqrt(Hz) at 1 kHz) is ~0.3.
    hwemu::sim::Vec3 a;
    auto accel = r->sim->Accel("turret_accel");
    for (int i = 0; i < 200; ++i) {
      r->sim->Step(0.001);
      const auto v = accel->Read();
      a.x += v.x / 200;
      a.y += v.y / 200;
      a.z += v.z / 200;
    }
    EXPECT_NEAR(a.z, 9.81, 0.3);
    EXPECT_NEAR(a.x, 0.0, 0.1);
    EXPECT_NEAR(a.y, 0.0, 0.1);
    EXPECT_NEAR(r->sim->Encoder("yaw_encoder")->Read().velocity, 0.0, 0.05);
    const auto att = r->sim->Attitude("turret_attitude")->Read();
    EXPECT_NEAR(att.roll, 0.0, 0.02);
    EXPECT_NEAR(att.pitch, 0.0, 0.02);
    EXPECT_NEAR(att.yaw, 0.0, 0.02);
    EXPECT_FALSE(r->sim->Switch("pitch_limit")->Read());
  }
}

TEST_F(Parity, TheSameVoltageSpinsTheYawTheSameWay) {
  Both([](Rig& r) {
    r.sim->Motor("yaw_motor")->Command(MotorCommand::Voltage(6.0));
    r.For(0.3);
  });
  const auto ea = achilles.sim->Encoder("yaw_encoder")->Read();
  const auto em = mujoco.sim->Encoder("yaw_encoder")->Read();
  ASSERT_GT(ea.velocity, 0.5) << "Achilles yaw didn't turn positive";
  ASSERT_GT(em.velocity, 0.5) << "MuJoCo yaw didn't turn positive";
  EXPECT_NEAR(em.velocity, ea.velocity, 0.1 * std::abs(ea.velocity));
  EXPECT_NEAR(em.position, ea.position, 0.1 * std::abs(ea.position) + 0.01);

  // The gyro and the attitude see it as a turn about +z.
  const double ga = achilles.sim->Gyro("turret_gyro")->Read().z;
  const double gm = mujoco.sim->Gyro("turret_gyro")->Read().z;
  EXPECT_NEAR(ga, ea.velocity, 0.05 * std::abs(ea.velocity) + 0.02);
  EXPECT_NEAR(gm, em.velocity, 0.05 * std::abs(em.velocity) + 0.02);
  EXPECT_NEAR(achilles.sim->Attitude("turret_attitude")->Read().yaw, ea.position, 0.02);
  EXPECT_NEAR(mujoco.sim->Attitude("turret_attitude")->Read().yaw, em.position, 0.02);

  // The winding current falls as the back-EMF rises, the same on both.
  const auto ma = achilles.sim->Motor("yaw_motor")->Read();
  const auto mm = mujoco.sim->Motor("yaw_motor")->Read();
  EXPECT_NEAR(mm.current, ma.current, 0.1 * std::abs(ma.current) + 0.05);
  ASSERT_TRUE(ma.temperature && mm.temperature);
  EXPECT_GT(*ma.temperature, 25.0);
  EXPECT_NEAR(*mm.temperature, *ma.temperature, 0.1);
}

TEST_F(Parity, ACurrentCommandPitchesUpAndClosesTheLimitSwitch) {
  Both([](Rig& r) {
    // Positive pitch about +y: nose down in a z-up world, the same on both.
    r.sim->Motor("pitch_motor")->Command(MotorCommand::Current(0.5));
    for (int i = 0; i < 2000 && r.sim->Encoder("pitch_encoder")->Read().position < 0.55; ++i) r.sim->Step(0.001);
  });
  for (Rig* r : {&achilles, &mujoco}) {
    SCOPED_TRACE(std::string(hwemu::sim::Name(r->sim->GetBackend())));
    EXPECT_GE(r->sim->Encoder("pitch_encoder")->Read().position, 0.5);
    EXPECT_TRUE(r->sim->Switch("pitch_limit")->Read());
    EXPECT_GT(r->sim->Attitude("turret_attitude")->Read().pitch, 0.45);
  }
  EXPECT_NEAR(mujoco.sim->Time(), achilles.sim->Time(), 0.1 * achilles.sim->Time());
}

TEST(MujocoCamera, SeesTheWallTwoMetresAheadAndReportsItsIntrinsics) {
  Rig r(Backend::kMujoco);
  auto cam = r.sim->Camera("d435");
  const auto in = cam->GetIntrinsics();
  EXPECT_EQ(in.width, 640U);
  EXPECT_EQ(in.height, 480U);
  EXPECT_NEAR(in.fy, 615.0, 0.5);
  r.For(0.05);
  hwemu::sim::CameraFrame f;
  ASSERT_TRUE(cam->Read(&f));
  ASSERT_EQ(f.depth.size(), 640U * 480U);
  // The camera sits at x = 0, the wall's face at x = 2.
  EXPECT_NEAR(f.depth[240 * 640 + 320], 2.0, 0.02);
  // The wall is grey-lit, not the black background.
  const auto* px = &f.rgb[3 * (240 * 640 + 320)];
  EXPECT_GT(px[0] + px[1] + px[2], 30);
}

// Ground truth (the hub's truth log): the same body, named the way each
// engine names it -- Achilles by its joint, MuJoCo by its body -- sits at
// the same world place, and an unknown name is an UnknownSensor. Only the
// place for the yaw: Achilles turns a revolute joint's frame to put x on
// its axis (tools/urdf_to_arow.py), so frames match in orientation only
// where nothing is turned -- the welded base here, a floating base on a
// robot.
TEST_F(Parity, BothGiveTheSameBodyPose) {
  Both([](Rig& r) { r.For(0.05); });
  const hwemu::sim::Pose a = achilles.sim->BodyPose("turret/yaw");
  const hwemu::sim::Pose m = mujoco.sim->BodyPose("yaw_link");
  EXPECT_NEAR(a.position.x, m.position.x, 1e-4);
  EXPECT_NEAR(a.position.y, m.position.y, 1e-4);
  EXPECT_NEAR(a.position.z, m.position.z, 1e-4);
  const hwemu::sim::Quat qa = achilles.sim->BodyPose("turret/base").orientation;
  const hwemu::sim::Quat qm = mujoco.sim->BodyPose("base_link").orientation;
  EXPECT_NEAR(std::abs((qa.w * qm.w) + (qa.x * qm.x) + (qa.y * qm.y) + (qa.z * qm.z)), 1.0, 1e-4);
  EXPECT_THROW((void)achilles.sim->BodyPose("turret/no_such_joint"), hwemu::sim::UnknownSensor);
  EXPECT_THROW((void)mujoco.sim->BodyPose("no_such_body"), hwemu::sim::UnknownSensor);
}

}  // namespace

#else
TEST(Parity, NeedsBothBackends) { GTEST_SKIP() << "built without Achilles or MuJoCo"; }
#endif
