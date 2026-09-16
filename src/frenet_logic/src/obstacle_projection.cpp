#include "lidar_path_planning/obstacle_projection.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lidar_path_planning {

std::vector<FrenetObstacle> projectObstaclesToFrenet(
    const ReferencePath& reference, const std::vector<Obstacle2d>& obstacles,
    double minimum_s, double maximum_s, double maximum_distance_m,
    double anchor_s, double station_tolerance_m) {
  std::vector<FrenetObstacle> output;
  output.reserve(obstacles.size());
  const ReferencePoint anchor = reference.sample(anchor_s);
  const double anchor_cosine = std::cos(anchor.yaw);
  const double anchor_sine = std::sin(anchor.yaw);
  for (const Obstacle2d& obstacle : obstacles) {
    const double expected_station =
        anchor_s + anchor_cosine * (obstacle.center.x - anchor.x) +
        anchor_sine * (obstacle.center.y - anchor.y);
    const double local_minimum_s =
        std::max(minimum_s, expected_station - station_tolerance_m);
    const double local_maximum_s =
        std::min(maximum_s, expected_station + station_tolerance_m);
    if (local_minimum_s > local_maximum_s) {
      continue;
    }
    const Projection center = reference.projectPoint(
        obstacle.center, local_minimum_s, local_maximum_s);
    if (!center.valid || center.distance_m > maximum_distance_m) {
      continue;
    }
    FrenetObstacle frenet;
    frenet.s_min = std::numeric_limits<double>::infinity();
    frenet.s_max = -std::numeric_limits<double>::infinity();
    frenet.d_min = std::numeric_limits<double>::infinity();
    frenet.d_max = -std::numeric_limits<double>::infinity();
    const double cosine = std::cos(center.reference_yaw);
    const double sine = std::sin(center.reference_yaw);
    for (const Point2d& corner : obstacle.corners) {
      const double dx = corner.x - obstacle.center.x;
      const double dy = corner.y - obstacle.center.y;
      const double corner_s = center.s + cosine * dx + sine * dy;
      const double corner_d = center.d - sine * dx + cosine * dy;
      frenet.s_min = std::min(frenet.s_min, corner_s);
      frenet.s_max = std::max(frenet.s_max, corner_s);
      frenet.d_min = std::min(frenet.d_min, corner_d);
      frenet.d_max = std::max(frenet.d_max, corner_d);
    }
    output.push_back(frenet);
  }
  return output;
}

}  // namespace lidar_path_planning
