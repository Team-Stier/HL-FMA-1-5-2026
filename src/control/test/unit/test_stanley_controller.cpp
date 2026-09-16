#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "control/lateral/stanley_controller.hpp"

namespace stier_control {
namespace {

constexpr double kPi = 3.14159265358979323846;

StanleyConfig t870Config() {
  StanleyConfig config;
  config.wheelbase_m = 0.75;
  config.gain = 0.8;
  config.softening_speed_mps = 0.5;
  config.minimum_control_speed_mps = 0.2;
  config.heading_window_m = 0.8;
  config.heading_error_gain = 1.0;
  config.curvature_feedforward_gain = 0.0;
  config.curvature_preview_distance_m = 1.5;
  config.yaw_rate_damping_gain_sec = 0.0;
  config.maximum_steering_angle_rad = 25.0 * kPi / 180.0;
  config.maximum_steering_rate_rad_per_sec = 45.0 * kPi / 180.0;
  return config;
}

TEST(StanleyController, StraightCenteredPathRequestsZeroSteering) {
  StanleyController controller(t870Config());
  const std::vector<Point2d> path{{0.0, 0.0}, {1.0, 0.0}, {3.0, 0.0}};
  const StanleyResult result =
      controller.calculate(path, 0.5, 0.0, 0.0, 0.05);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_NEAR(0.0, result.cross_track_error_m, 1.0e-12);
  EXPECT_NEAR(0.0, result.heading_error_rad, 1.0e-12);
  EXPECT_NEAR(0.0, result.steering_angle_rad, 1.0e-12);
  EXPECT_NEAR(0.75, result.target.x, 1.0e-12);
}

TEST(StanleyController, PathToLeftRequestsPositiveRoadWheelAngle) {
  StanleyController controller(t870Config());
  const std::vector<Point2d> path{{0.0, 0.5}, {1.0, 0.5}, {3.0, 0.5}};
  const StanleyResult result =
      controller.calculate(path, 0.5, 0.0, 0.0, 0.05);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_GT(result.cross_track_error_m, 0.0);
  EXPECT_GT(result.requested_steering_angle_rad, 0.0);
  EXPECT_GT(result.steering_angle_rad, 0.0);
}

TEST(StanleyController, AppliesConfiguredSteeringRateLimit) {
  StanleyConfig config = t870Config();
  config.maximum_steering_rate_rad_per_sec = 10.0 * kPi / 180.0;
  StanleyController controller(config);
  const std::vector<Point2d> path{{0.0, 2.0}, {1.0, 2.0}, {3.0, 2.0}};
  const StanleyResult result =
      controller.calculate(path, 0.2, 0.0, 0.0, 0.1);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_GT(result.requested_steering_angle_rad, 1.0 * kPi / 180.0);
  EXPECT_NEAR(1.0 * kPi / 180.0, result.steering_angle_rad, 1.0e-12);
}

TEST(StanleyController, RejectsPathWithoutSegment) {
  StanleyController controller(t870Config());
  const StanleyResult result =
      controller.calculate({{0.0, 0.0}}, 0.5, 0.0, 0.0, 0.05);
  EXPECT_FALSE(result.valid);
}

}  // namespace
}  // namespace stier_control
