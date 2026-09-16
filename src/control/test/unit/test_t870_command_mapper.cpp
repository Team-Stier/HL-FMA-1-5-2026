#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "control/vehicle/t870_command_mapper.hpp"

namespace stier_control {
namespace {

constexpr double kPi = 3.14159265358979323846;

T870SteeringCommandConfig t870Config() {
  T870SteeringCommandConfig config;
  config.road_wheel_angle_at_command_limit_rad = 25.0 * kPi / 180.0;
  config.command_limit_deg = 25;
  config.steering_command_sign = -1;
  return config;
}

TEST(T870CommandMapper, MapsRep103LeftTurnToUnoNegativeCommand) {
  EXPECT_EQ(-25, roadWheelAngleToT870CommandDeg(
                     25.0 * kPi / 180.0, t870Config()));
  EXPECT_EQ(25, roadWheelAngleToT870CommandDeg(
                    -25.0 * kPi / 180.0, t870Config()));
  EXPECT_EQ(0, roadWheelAngleToT870CommandDeg(0.0, t870Config()));
}

TEST(T870CommandMapper, SaturatesBeyondMeasuredRoadWheelRange) {
  EXPECT_EQ(-25, roadWheelAngleToT870CommandDeg(
                     60.0 * kPi / 180.0, t870Config()));
  EXPECT_EQ(25, roadWheelAngleToT870CommandDeg(
                    -60.0 * kPi / 180.0, t870Config()));
}

TEST(T870CommandMapper, RejectsInvalidInputAndConfiguration) {
  T870SteeringCommandConfig config = t870Config();
  config.steering_command_sign = 0;
  EXPECT_THROW(roadWheelAngleToT870CommandDeg(0.0, config),
               std::invalid_argument);
  EXPECT_THROW(roadWheelAngleToT870CommandDeg(
                   std::numeric_limits<double>::quiet_NaN(), t870Config()),
               std::invalid_argument);
}

}  // namespace
}  // namespace stier_control
