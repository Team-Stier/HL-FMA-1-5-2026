#pragma once

#include <vector>

#include "lidar_path_planning/planner_backend.hpp"

namespace lidar_path_planning {

struct FrenetPlannerConfig {
  VehicleGeometry vehicle;
  double horizon_m{8.0};
  double sample_interval_m{0.10};
  double lateral_sample_interval_m{0.25};
  double maximum_lateral_offset_m{1.50};
  std::vector<double> midpoint_fractions{0.30, 0.50, 0.70};
  double collision_margin_m{0.15};
  double boundary_margin_m{0.10};
  double maximum_curvature_per_m{0.48};
  double maximum_curvature_rate_per_m2{0.80};
  double maximum_lateral_slope{0.80};
  double minimum_longitudinal_scale{0.20};
  double boundary_perimeter_sample_interval_m{0.10};
  double weight_deviation{1.0};
  double weight_smoothness{8.0};
  double weight_jerk{1.0};
  double weight_clearance{10.0};
  double weight_terminal_offset{4.0};
  double weight_previous_path{3.0};
};

bool isValid(const FrenetPlannerConfig& config, std::string* reason);

class FrenetPlanner final : public PlannerBackend {
 public:
  explicit FrenetPlanner(const FrenetPlannerConfig& config);

  std::string name() const override { return "frenet"; }
  void reset() override { previous_path_.clear(); }
  PlannerResult plan(const ReferencePath& reference,
                     const PlannerInput& input) override;

 private:
  struct Candidate;
  FrenetPlannerConfig config_;
  std::vector<PathPoint> previous_path_;
};

bool interpolatePathLateralOffset(const std::vector<PathPoint>& path,
                                  double route_s, double* lateral_offset);

}  // namespace lidar_path_planning
