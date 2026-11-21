// The hub's actors: where hub::ActorPose puts a scripted body over time.
#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <stdexcept>

#include "hwemu/hub/actor.hpp"
#include "hwemu/sim/fake_sim.hpp"

namespace {

using hwemu::config::ActorSpec;
using hwemu::hub::ActorPose;

double Yaw(const hwemu::sim::Pose& p) { return 2.0 * std::atan2(p.orientation.z, p.orientation.w); }

// A 2 x 1 m rectangle, counter-clockwise from the origin: 6 m around.
ActorSpec Rectangle() {
  ActorSpec a;
  a.name = "box";
  a.path = {{0, 0}, {2, 0}, {2, 1}, {0, 1}};
  a.speed = 1.0;
  a.z = 0.1;
  a.turn = 0.2;
  return a;
}

TEST(Actor, WaitsAtTheFirstPointUntilItsStart) {
  ActorSpec a = Rectangle();
  a.start = 5.0;
  const auto p = ActorPose(a, 3.0);
  EXPECT_DOUBLE_EQ(p.position.x, 0.0);
  EXPECT_DOUBLE_EQ(p.position.y, 0.0);
  EXPECT_DOUBLE_EQ(p.position.z, 0.1);
  EXPECT_NEAR(ActorPose(a, 6.0).position.x, 1.0, 1e-9);
}

TEST(Actor, GoesAroundTheLoopAtItsSpeedAndComesBack) {
  const ActorSpec a = Rectangle();
  auto p = ActorPose(a, 1.0);  // halfway along the first leg
  EXPECT_NEAR(p.position.x, 1.0, 1e-9);
  EXPECT_NEAR(p.position.y, 0.0, 1e-9);
  p = ActorPose(a, 2.5);  // halfway up the second
  EXPECT_NEAR(p.position.x, 2.0, 1e-9);
  EXPECT_NEAR(p.position.y, 0.5, 1e-9);
  p = ActorPose(a, 5.5);  // halfway down the last, back to the start
  EXPECT_NEAR(p.position.x, 0.0, 1e-9);
  EXPECT_NEAR(p.position.y, 0.5, 1e-9);
  p = ActorPose(a, 7.0);  // a lap later
  EXPECT_NEAR(p.position.x, 1.0, 1e-9);
  EXPECT_NEAR(p.position.y, 0.0, 1e-9);
}

TEST(Actor, FacesAlongEachLegAndTurnsHalfwayAtAWaypoint) {
  const ActorSpec a = Rectangle();
  EXPECT_NEAR(Yaw(ActorPose(a, 1.0)), 0.0, 1e-9);
  EXPECT_NEAR(Yaw(ActorPose(a, 2.5)), std::numbers::pi / 2, 1e-9);
  EXPECT_NEAR(Yaw(ActorPose(a, 2.0)), std::numbers::pi / 4, 1e-9);   // at the corner: halfway round
  EXPECT_NEAR(Yaw(ActorPose(a, 1.9)), std::numbers::pi / 8, 1e-9);   // easing in, 0.1 m before it
  EXPECT_NEAR(Yaw(ActorPose(a, 0.0)), -std::numbers::pi / 4, 1e-9);  // the last leg's turn into the first
}

TEST(Actor, WithASpinTurnsAtItsRateWherever) {
  ActorSpec a = Rectangle();
  a.spin = 0.5;
  EXPECT_NEAR(Yaw(ActorPose(a, 1.0)), 0.5, 1e-9);
  EXPECT_NEAR(Yaw(ActorPose(a, 2.5)), 1.25, 1e-9);
}

TEST(Actor, RefusesAPathOrSpeedItCantFollow) {
  ActorSpec a = Rectangle();
  a.path = {{1, 1}};
  EXPECT_THROW((void)ActorPose(a, 0.0), std::invalid_argument);
  a.path = {{1, 1}, {1, 1}};
  EXPECT_THROW((void)ActorPose(a, 0.0), std::invalid_argument);
  a = Rectangle();
  a.speed = 0.0;
  EXPECT_THROW((void)ActorPose(a, 0.0), std::invalid_argument);
}

TEST(Actor, TheFakeSimHoldsWhatItsPlaced) {
  hwemu::sim::FakeSim sim;
  EXPECT_THROW((void)sim.BodyPose("box"), hwemu::sim::UnknownSensor);
  sim.SetBodyPose("box", ActorPose(Rectangle(), 1.0));
  EXPECT_NEAR(sim.BodyPose("box").position.x, 1.0, 1e-9);
}

}  // namespace
