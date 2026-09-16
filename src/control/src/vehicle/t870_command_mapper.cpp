#include "control/vehicle/t870_command_mapper.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace stier_control {

bool isValidT870SteeringCommandConfig(
    const T870SteeringCommandConfig& config) {
  return std::isfinite(config.road_wheel_angle_at_command_limit_rad) &&
         config.road_wheel_angle_at_command_limit_rad > 0.0 &&
         config.command_limit_deg > 0 &&
         (config.steering_command_sign == -1 ||
          config.steering_command_sign == 1);
}

int16_t roadWheelAngleToT870CommandDeg(
    double road_wheel_angle_rad,
    const T870SteeringCommandConfig& config) {
  if (!isValidT870SteeringCommandConfig(config)) {
    throw std::invalid_argument("invalid T870 steering command configuration");
  }
  if (!std::isfinite(road_wheel_angle_rad)) {
    throw std::invalid_argument("road wheel angle must be finite");
  }

  const double normalized = std::max(
      -1.0,
      std::min(1.0, road_wheel_angle_rad /
                        config.road_wheel_angle_at_command_limit_rad));
  const double command_deg =
      static_cast<double>(config.steering_command_sign) * normalized *
      static_cast<double>(config.command_limit_deg);
  const long rounded = std::lround(command_deg);
  return static_cast<int16_t>(std::max<long>(
      -config.command_limit_deg,
      std::min<long>(config.command_limit_deg, rounded)));
}

}  // namespace stier_control
