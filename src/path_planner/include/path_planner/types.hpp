#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace path_planner {

constexpr double kPi = 3.14159265358979323846;

struct Point2d {
  double x{0.0};
  double y{0.0};
};

struct Pose2d {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Waypoint {
  double x{0.0};
  double y{0.0};
  double left_bound_m{0.0};
  double right_bound_m{0.0};
  double speed_limit_kph{0.0};
  std::string mission_zone;
};

struct ReferencePoint : Waypoint {
  double s{0.0};
  double yaw{0.0};
  double curvature{0.0};
};

struct Projection {
  bool valid{false};
  double s{0.0};
  double d{0.0};
  double distance_m{0.0};
  double reference_yaw{0.0};
  std::size_t segment_index{0U};
};

struct Obstacle2d {
  Point2d center;
  std::array<Point2d, 4U> corners{};
};

struct FrenetObstacle {
  double s_min{0.0};
  double s_max{0.0};
  double d_min{0.0};
  double d_max{0.0};
};

struct PathPoint {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double curvature{0.0};
  double s{0.0};
  double d{0.0};
};

struct VehicleGeometry {
  double length_m{0.0};
  double width_m{0.0};
  double wheelbase_m{0.0};
  double rear_axle_to_center_m{0.0};
};

struct PlannerInput {
  Pose2d pose;
  Projection projection;
  std::vector<Obstacle2d> obstacles;
  std::vector<FrenetObstacle> frenet_obstacles;
};

struct PlannerResult {
  bool valid{false};
  std::vector<PathPoint> path;
  std::string reason;
  std::size_t evaluated_candidates{0U};
  std::size_t feasible_candidates{0U};
};

double normalizeAngle(double angle);
double interpolateAngle(double from, double to, double ratio);
double squaredDistance(const Point2d& first, const Point2d& second);

}  // namespace path_planner
