#pragma once

#include <cstdint>

namespace stier_control {

// Upper-level longitudinal policy only. The Arduino controller owns the
// encoder speed PI loop, PWM saturation, and actuator ramping.
struct FixedSpeedConfig {
  int target_speed_kph{0};
  int maximum_speed_kph{0};
};

bool isValidFixedSpeedConfig(const FixedSpeedConfig& config);

class FixedSpeedController {
 public:
  explicit FixedSpeedController(const FixedSpeedConfig& config);

  uint16_t commandKph() const;

 private:
  uint16_t command_kph_{0U};
};

}  // namespace stier_control
