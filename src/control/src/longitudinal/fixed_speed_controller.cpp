#include "control/longitudinal/fixed_speed_controller.hpp"

#include <limits>
#include <stdexcept>

namespace stier_control {

bool isValidFixedSpeedConfig(const FixedSpeedConfig& config) {
  return config.target_speed_kph >= 0 && config.maximum_speed_kph > 0 &&
         config.target_speed_kph <= config.maximum_speed_kph &&
         config.maximum_speed_kph <=
             static_cast<int>(std::numeric_limits<uint16_t>::max());
}

FixedSpeedController::FixedSpeedController(const FixedSpeedConfig& config) {
  if (!isValidFixedSpeedConfig(config)) {
    throw std::invalid_argument("invalid fixed-speed configuration");
  }
  command_kph_ = static_cast<uint16_t>(config.target_speed_kph);
}

uint16_t FixedSpeedController::commandKph() const {
  return command_kph_;
}

}  // namespace stier_control
