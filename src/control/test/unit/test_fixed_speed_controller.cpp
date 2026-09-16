#include <stdexcept>

#include <gtest/gtest.h>

#include "control/longitudinal/fixed_speed_controller.hpp"

namespace stier_control {
namespace {

TEST(FixedSpeedController, PublishesConfiguredTargetBelowUpperLimit) {
  const FixedSpeedController controller({5, 10});
  EXPECT_EQ(5U, controller.commandKph());
}

TEST(FixedSpeedController, AllowsStoppedTarget) {
  const FixedSpeedController controller({0, 10});
  EXPECT_EQ(0U, controller.commandKph());
}

TEST(FixedSpeedController, RejectsTargetAboveUpperLimit) {
  EXPECT_THROW(FixedSpeedController({11, 10}), std::invalid_argument);
}

TEST(FixedSpeedController, RejectsInvalidUpperLimit) {
  EXPECT_THROW(FixedSpeedController({0, 0}), std::invalid_argument);
  EXPECT_THROW(FixedSpeedController({5, 65536}), std::invalid_argument);
}

}  // namespace
}  // namespace stier_control
