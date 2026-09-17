#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "control/lateral/pure_pursuit.hpp"

namespace stier_control {
namespace {

constexpr double kPi = 3.14159265358979323846;

PurePursuitConfig t870Config() {
  PurePursuitConfig config;
  config.wheelbase_m = 0.75;
  config.lookahead_base_m = 1.0;
  config.lookahead_speed_gain_sec = 0.5;
  config.lookahead_curvature_gain_m = 1.0;
  config.lookahead_min_m = 0.8;
  config.lookahead_max_m = 1.8;
  config.minimum_target_distance_m = 0.2;
  config.maximum_steering_angle_rad = 25.0 * kPi / 180.0;
  return config;
}

TEST(PurePursuit, FarLateralPathUsesNearbyForwardRecoveryTarget) {
  for (double y : {-5.0, 5.0}) {
    const auto result = computePurePursuit({{-2.0, y}, {20.0, y}},
                                         0.5, 0.0, t870Config());
    ASSERT_TRUE(result.valid);
    EXPECT_NEAR(result.target.x, result.lookahead_m, 1e-9);
    EXPECT_NEAR(result.target.y, y, 1e-9);
    EXPECT_GT(result.steering_angle_rad * y, 0.0);
  }
}

TEST(PurePursuit, RecoveryNeverTargetsBehindVehicle) {
  EXPECT_FALSE(computePurePursuit({{-20.0, 5.0}, {-2.0, 5.0}},
                                 0.5, 0.0, t870Config()).valid);
  EXPECT_FALSE(computePurePursuit({{0.0, 5.0}, {-20.0, 5.0}},
                                 0.5, 0.0, t870Config()).valid);
}

TEST(PurePursuit, StraightPathRequestsZeroSteering) {
  const std::vector<Point2d> path{{0.0, 0.0}, {1.0, 0.0}, {3.0, 0.0}};
  const PurePursuitResult result =
      computePurePursuit(path, 0.5, 0.0, t870Config());

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(0.0, result.steering_angle_rad, 1.0e-12);
  EXPECT_GT(result.target.x, 0.0);
}

TEST(PurePursuit, PathToLeftRequestsPositiveRoadWheelAngle) {
  const std::vector<Point2d> path{{0.0, 0.5}, {1.0, 0.5}, {3.0, 0.5}};
  const PurePursuitResult result =
      computePurePursuit(path, 0.5, 0.0, t870Config());

  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.steering_angle_rad, 0.0);
  EXPECT_LE(result.steering_angle_rad, 25.0 * kPi / 180.0);
}

TEST(PurePursuit, CurvatureShortensAdaptiveLookahead) {
  const std::vector<Point2d> path{{0.0, 0.0}, {1.0, 0.0}, {3.0, 0.0}};
  const PurePursuitResult straight =
      computePurePursuit(path, 0.8, 0.0, t870Config());
  const PurePursuitResult curve =
      computePurePursuit(path, 0.8, 1.0, t870Config());

  ASSERT_TRUE(straight.valid);
  ASSERT_TRUE(curve.valid);
  EXPECT_LT(curve.lookahead_m, straight.lookahead_m);
  EXPECT_GE(curve.lookahead_m, t870Config().lookahead_min_m);
}

TEST(PurePursuit, RejectsNonFiniteRuntimeInput) {
  const std::vector<Point2d> path{{0.0, 0.0}, {1.0, 0.0}};
  EXPECT_FALSE(computePurePursuit(
                   path, std::numeric_limits<double>::quiet_NaN(), 0.0,
                   t870Config())
                   .valid);
}

}  // namespace
}  // namespace stier_control
