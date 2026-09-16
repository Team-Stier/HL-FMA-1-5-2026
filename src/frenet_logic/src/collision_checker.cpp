#include "lidar_path_planning/collision_checker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lidar_path_planning {
namespace {

bool overlapsOnAxis(const Polygon4& first, const Polygon4& second,
                    const Point2d& axis) {
  const double norm = std::hypot(axis.x, axis.y);
  if (norm <= 1.0e-12) {
    return true;
  }
  const double axis_x = axis.x / norm;
  const double axis_y = axis.y / norm;
  double first_minimum = std::numeric_limits<double>::infinity();
  double first_maximum = -std::numeric_limits<double>::infinity();
  double second_minimum = std::numeric_limits<double>::infinity();
  double second_maximum = -std::numeric_limits<double>::infinity();
  for (const Point2d& point : first) {
    const double projection = point.x * axis_x + point.y * axis_y;
    first_minimum = std::min(first_minimum, projection);
    first_maximum = std::max(first_maximum, projection);
  }
  for (const Point2d& point : second) {
    const double projection = point.x * axis_x + point.y * axis_y;
    second_minimum = std::min(second_minimum, projection);
    second_maximum = std::max(second_maximum, projection);
  }
  constexpr double kContactTolerance = 1.0e-9;
  return first_maximum + kContactTolerance >= second_minimum &&
         second_maximum + kContactTolerance >= first_minimum;
}

bool boundingBoxesOverlap(const Polygon4& first, const Polygon4& second) {
  const auto bounds = [](const Polygon4& polygon) {
    std::array<double, 4U> result{{
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
    }};
    for (const Point2d& point : polygon) {
      result[0U] = std::min(result[0U], point.x);
      result[1U] = std::max(result[1U], point.x);
      result[2U] = std::min(result[2U], point.y);
      result[3U] = std::max(result[3U], point.y);
    }
    return result;
  };
  const std::array<double, 4U> first_bounds = bounds(first);
  const std::array<double, 4U> second_bounds = bounds(second);
  constexpr double kContactTolerance = 1.0e-9;
  return first_bounds[1U] + kContactTolerance >= second_bounds[0U] &&
         second_bounds[1U] + kContactTolerance >= first_bounds[0U] &&
         first_bounds[3U] + kContactTolerance >= second_bounds[2U] &&
         second_bounds[3U] + kContactTolerance >= first_bounds[2U];
}

bool hasSeparatingAxis(const Polygon4& axes_from, const Polygon4& first,
                       const Polygon4& second) {
  for (std::size_t index = 0U; index < axes_from.size(); ++index) {
    const Point2d& start = axes_from[index];
    const Point2d& end = axes_from[(index + 1U) % axes_from.size()];
    const Point2d normal{-(end.y - start.y), end.x - start.x};
    if (!overlapsOnAxis(first, second, normal)) {
      return true;
    }
  }
  return false;
}

}  // namespace

Polygon4 vehicleFootprint(const Pose2d& rear_axle_pose,
                          const VehicleGeometry& vehicle,
                          double margin_m) {
  const double half_length = vehicle.length_m / 2.0 + margin_m;
  const double half_width = vehicle.width_m / 2.0 + margin_m;
  const double cosine = std::cos(rear_axle_pose.yaw);
  const double sine = std::sin(rear_axle_pose.yaw);
  const double center_x =
      rear_axle_pose.x + vehicle.rear_axle_to_center_m * cosine;
  const double center_y =
      rear_axle_pose.y + vehicle.rear_axle_to_center_m * sine;
  const auto to_world = [&](double longitudinal, double lateral) -> Point2d {
    return {center_x + cosine * longitudinal - sine * lateral,
            center_y + sine * longitudinal + cosine * lateral};
  };
  return {{to_world(-half_length, -half_width),
           to_world(half_length, -half_width),
           to_world(half_length, half_width),
           to_world(-half_length, half_width)}};
}

bool convexPolygonsIntersect(const Polygon4& first, const Polygon4& second) {
  return boundingBoxesOverlap(first, second) &&
         !hasSeparatingAxis(first, first, second) &&
         !hasSeparatingAxis(second, first, second);
}

bool vehicleIntersectsObstacle(const Pose2d& rear_axle_pose,
                               const VehicleGeometry& vehicle,
                               double margin_m,
                               const Obstacle2d& obstacle) {
  return convexPolygonsIntersect(
      vehicleFootprint(rear_axle_pose, vehicle, margin_m), obstacle.corners);
}

bool pathHasCollision(const std::vector<PathPoint>& path,
                      const std::vector<Obstacle2d>& obstacles,
                      const VehicleGeometry& vehicle, double margin_m,
                      double maximum_translation_step_m,
                      double maximum_yaw_step_rad,
                      std::size_t* collision_index) {
  if (path.empty() || obstacles.empty()) {
    return false;
  }
  if (!std::isfinite(maximum_translation_step_m) ||
      !std::isfinite(maximum_yaw_step_rad) ||
      maximum_translation_step_m <= 0.0 || maximum_yaw_step_rad <= 0.0) {
    if (collision_index != nullptr) {
      *collision_index = 0U;
    }
    return true;
  }

  const auto pose_collides = [&](const Pose2d& pose) {
    const Polygon4 footprint = vehicleFootprint(pose, vehicle, margin_m);
    for (const Obstacle2d& obstacle : obstacles) {
      if (convexPolygonsIntersect(footprint, obstacle.corners)) {
        return true;
      }
    }
    return false;
  };

  if (pose_collides({path.front().x, path.front().y, path.front().yaw})) {
    if (collision_index != nullptr) {
      *collision_index = 0U;
    }
    return true;
  }
  for (std::size_t index = 1U; index < path.size(); ++index) {
    const PathPoint& first = path[index - 1U];
    const PathPoint& second = path[index];
    const double translation = std::hypot(second.x - first.x,
                                          second.y - first.y);
    const double yaw_change =
        std::abs(normalizeAngle(second.yaw - first.yaw));
    const std::size_t translation_steps = static_cast<std::size_t>(
        std::ceil(translation / maximum_translation_step_m));
    const std::size_t yaw_steps = static_cast<std::size_t>(
        std::ceil(yaw_change / maximum_yaw_step_rad));
    const std::size_t steps =
        std::max<std::size_t>(1U, std::max(translation_steps, yaw_steps));
    for (std::size_t step = 1U; step <= steps; ++step) {
      const double ratio = static_cast<double>(step) /
                           static_cast<double>(steps);
      const Pose2d pose{
          first.x + ratio * (second.x - first.x),
          first.y + ratio * (second.y - first.y),
          interpolateAngle(first.yaw, second.yaw, ratio),
      };
      if (pose_collides(pose)) {
        if (collision_index != nullptr) {
          *collision_index = index;
        }
        return true;
      }
    }
  }
  return false;
}

}  // namespace lidar_path_planning
