#pragma once

#include <vector>

namespace stier_control {

struct Point2d {
  double x{0.0};
  double y{0.0};
};

struct PurePursuitConfig {
  double wheelbase_m{0.0};
  double lookahead_base_m{0.0};
  double lookahead_speed_gain_sec{0.0};
  double lookahead_curvature_gain_m{0.0};
  double lookahead_min_m{0.0};
  double lookahead_max_m{0.0};
  double minimum_target_distance_m{0.0};
  double maximum_steering_angle_rad{0.0};
};

struct PurePursuitResult {
  bool valid{false};
  Point2d target;
  double lookahead_m{0.0};
  double steering_angle_rad{0.0};
};

PurePursuitResult computePurePursuit(
    const std::vector<Point2d>& path_in_rear_axle_frame,
    double longitudinal_speed_mps, double preview_curvature_m_inv,
    const PurePursuitConfig& config);

}  // namespace stier_control
