#pragma once

#include "path_planner/reference_path.hpp"
#include "path_planner/types.hpp"

namespace path_planner {

// Checks sampled points along the complete, oriented vehicle perimeter against
// the left/right bounds at their own route stations. The station window keeps
// a footprint on an S/parallel section from projecting onto another branch.
bool vehicleWithinReferenceBounds(const ReferencePath& reference,
                                  const Pose2d& rear_axle_pose,
                                  const VehicleGeometry& vehicle,
                                  double boundary_margin_m,
                                  double station_hint_s,
                                  double station_window_m,
                                  double perimeter_sample_interval_m);

// Also checks interpolated poses between path points. Sampling is a numerical
// approximation; the map must resolve small curb features and margins must
// include map, localization, tracking and sampling error.
bool sweptPathWithinReferenceBounds(
    const ReferencePath& reference, const std::vector<PathPoint>& path,
    const VehicleGeometry& vehicle, double boundary_margin_m,
    double perimeter_sample_interval_m, double maximum_translation_step_m,
    double maximum_yaw_step_rad, std::size_t* violation_index = nullptr);

}  // namespace path_planner
