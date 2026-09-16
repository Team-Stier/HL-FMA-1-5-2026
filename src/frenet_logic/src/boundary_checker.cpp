#include "lidar_path_planning/boundary_checker.hpp"

#include <algorithm>
#include <cmath>

#include "lidar_path_planning/collision_checker.hpp"

namespace lidar_path_planning {

bool vehicleWithinReferenceBounds(const ReferencePath& reference,
                                  const Pose2d& rear_axle_pose,
                                  const VehicleGeometry& vehicle,
                                  double boundary_margin_m,
                                  double station_hint_s,
                                  double station_window_m,
                                  double perimeter_sample_interval_m) {
  if (reference.empty() || !std::isfinite(rear_axle_pose.x) ||
      !std::isfinite(rear_axle_pose.y) || !std::isfinite(rear_axle_pose.yaw) ||
      !std::isfinite(vehicle.length_m) || vehicle.length_m <= 0.0 ||
      !std::isfinite(vehicle.width_m) || vehicle.width_m <= 0.0 ||
      !std::isfinite(vehicle.rear_axle_to_center_m) ||
      !std::isfinite(boundary_margin_m) || boundary_margin_m < 0.0 ||
      !std::isfinite(station_hint_s) ||
      !std::isfinite(station_window_m) || station_window_m <= 0.0 ||
      !std::isfinite(perimeter_sample_interval_m) ||
      perimeter_sample_interval_m <= 0.0) {
    return false;
  }

  const Polygon4 footprint =
      vehicleFootprint(rear_axle_pose, vehicle, boundary_margin_m);
  const double minimum_s = std::max(0.0, station_hint_s - station_window_m);
  const double maximum_s =
      std::min(reference.length(), station_hint_s + station_window_m);

  for (std::size_t edge = 0U; edge < footprint.size(); ++edge) {
    const Point2d& first = footprint[edge];
    const Point2d& second = footprint[(edge + 1U) % footprint.size()];
    const double edge_length = std::hypot(second.x - first.x,
                                          second.y - first.y);
    const std::size_t sample_count = std::max<std::size_t>(
        1U, static_cast<std::size_t>(
                std::ceil(edge_length / perimeter_sample_interval_m)));
    for (std::size_t sample = 0U; sample < sample_count; ++sample) {
      const double ratio = static_cast<double>(sample) /
                           static_cast<double>(sample_count);
      const Point2d point{first.x + ratio * (second.x - first.x),
                          first.y + ratio * (second.y - first.y)};
      const Projection projection =
          reference.projectPoint(point, minimum_s, maximum_s);
      if (!projection.valid) {
        return false;
      }
      const ReferencePoint bounds = reference.sample(projection.s);
      constexpr double kBoundaryTolerance = 1.0e-9;
      if (projection.d > bounds.left_bound_m + kBoundaryTolerance ||
          projection.d < -bounds.right_bound_m - kBoundaryTolerance) {
        return false;
      }
    }
  }
  return true;
}

bool sweptPathWithinReferenceBounds(
    const ReferencePath& reference, const std::vector<PathPoint>& path,
    const VehicleGeometry& vehicle, double boundary_margin_m,
    double perimeter_sample_interval_m, double maximum_translation_step_m,
    double maximum_yaw_step_rad, std::size_t* violation_index) {
  if (violation_index != nullptr) {
    *violation_index = 0U;
  }
  if (path.empty() || !std::isfinite(maximum_translation_step_m) ||
      maximum_translation_step_m <= 0.0 ||
      !std::isfinite(maximum_yaw_step_rad) || maximum_yaw_step_rad <= 0.0) {
    return false;
  }
  for (const PathPoint& point : path) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.yaw) || !std::isfinite(point.s)) {
      return false;
    }
  }
  const double station_window = vehicle.length_m + 2.0 * boundary_margin_m;
  const auto within = [&](const Pose2d& pose, double station) {
    return vehicleWithinReferenceBounds(
        reference, pose, vehicle, boundary_margin_m, station, station_window,
        perimeter_sample_interval_m);
  };
  if (!within({path.front().x, path.front().y, path.front().yaw},
              path.front().s)) {
    return false;
  }
  for (std::size_t index = 1U; index < path.size(); ++index) {
    const PathPoint& first = path[index - 1U];
    const PathPoint& second = path[index];
    const double translation = std::hypot(second.x - first.x,
                                          second.y - first.y);
    const double yaw_change =
        std::abs(normalizeAngle(second.yaw - first.yaw));
    const double steps_needed = std::max(
        std::ceil(std::max(translation, std::abs(second.s - first.s)) /
                  maximum_translation_step_m),
        std::ceil(yaw_change / maximum_yaw_step_rad));
    // Reject unreasonable inputs before converting doubles to size_t.
    if (!std::isfinite(steps_needed) || steps_needed > 100000.0) {
      return false;
    }
    const std::size_t steps =
        std::max<std::size_t>(1U, static_cast<std::size_t>(steps_needed));
    for (std::size_t step = 1U; step <= steps; ++step) {
      const double ratio = static_cast<double>(step) / steps;
      const Pose2d pose{first.x + ratio * (second.x - first.x),
                        first.y + ratio * (second.y - first.y),
                        interpolateAngle(first.yaw, second.yaw, ratio)};
      if (!within(pose, first.s + ratio * (second.s - first.s))) {
        if (violation_index != nullptr) {
          *violation_index = index;
        }
        return false;
      }
    }
  }
  return true;
}

}  // namespace lidar_path_planning
