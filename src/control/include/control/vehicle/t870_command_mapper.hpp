#pragma once

#include <cstdint>

namespace stier_control {

struct T870SteeringCommandConfig {
  double road_wheel_angle_at_command_limit_rad{0.0};
  int16_t command_limit_deg{25};
  int steering_command_sign{-1};
};

bool isValidT870SteeringCommandConfig(
    const T870SteeringCommandConfig& config);

int16_t roadWheelAngleToT870CommandDeg(
    double road_wheel_angle_rad,
    const T870SteeringCommandConfig& config);

}  // namespace stier_control
