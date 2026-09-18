#pragma once

#include <vector>
#include "control/lateral/pure_pursuit.hpp"

namespace stier_control {

// Planning assumptions, not measured tyre/brake limits. Tune on the vehicle.
// This policy changes speed only; mission/obstacle/EStop authorization is owned
// by the existing nodes. The Arduino interface has integer km/h resolution.
struct PathSpeedConfig {
  double wheelbase_m{0.75};
  double lateral_acceleration_mps2{1.0};
  double preview_deceleration_mps2{0.25};
  double reaction_time_sec{0.5};
  double preview_distance_m{40.0};
  double sample_interval_m{0.5};
};

struct PathSpeedResult {
  double speed_mps{0.0};
  double curve_limit_mps{0.0};
  double stop_limit_mps{0.0};
  double visibility_limit_mps{0.0};
  double maximum_curvature_m_inv{0.0};
};

bool isValidPathSpeedConfig(const PathSpeedConfig& config);

// Path is in travel-aligned rear-axle coordinates (reverse X already flipped).
// requested/applied angles are road-wheel radians, NEVER raw feedback ADC.
// A negative remaining_stop_m means no scheduled stop. At an authorized drive
// tick the profile does not introduce a brake or neutral command.
PathSpeedResult computePathSpeed(
    const std::vector<Point2d>& path, double measured_speed_mps,
    double requested_steering_rad, double applied_steering_rad,
    double ceiling_mps, double remaining_stop_m, const PathSpeedConfig& config);

}  // namespace stier_control
