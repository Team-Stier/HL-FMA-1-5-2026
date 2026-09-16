#include "path_planner/boundary_checker.hpp"
#include "path_planner/collision_checker.hpp"
#include "path_planner/frenet_planner.hpp"
#include "path_planner/obstacle_projection.hpp"
#include "path_planner/pose_jump_guard.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace lp = path_planner;
#define CHECK(condition) do { if (!(condition)) throw std::runtime_error( \
    std::string(__func__) + ":" + std::to_string(__LINE__) + " " #condition); } while (false)
lp::ReferencePath reference() {
  std::vector<lp::Waypoint> points;
  for (int i = -20; i <= 200; ++i) points.push_back({i * .1, 0, 3.27, 3.27, 2.0, "SYNTHETIC"});
  lp::ReferencePath result;
  std::string error;
  CHECK(result.initialize(points, &error));
  return result;
}
lp::FrenetPlannerConfig config() {
  lp::FrenetPlannerConfig value;
  value.vehicle = {1.40, .775, .75, .38};  // synthetic fixture, NOT measured calibration
  return value;
}
lp::Obstacle2d obstacle(double x, double y, double length, double width) {
  return {{x, y}, {{{x-length/2, y-width/2}, {x+length/2, y-width/2},
                    {x+length/2, y+width/2}, {x-length/2, y+width/2}}}};
}
lp::PlannerInput input(const lp::ReferencePath& ref, std::vector<lp::Obstacle2d> obstacles = {}) {
  lp::PlannerInput value;
  value.pose = {0, 0, 0};
  value.projection = ref.projectPose(value.pose, 0, 10, 2);
  value.obstacles = obstacles;
  value.frenet_obstacles = lp::projectObstaclesToFrenet(ref, obstacles, 0, 12, 5, value.projection.s, 3);
  return value;
}
void clearRoad() {
  const auto ref = reference(); lp::FrenetPlanner planner(config());
  const auto output = planner.plan(ref, input(ref));
  CHECK(output.valid); CHECK(output.path.size() > 20);
  for (const auto& p : output.path) CHECK(std::abs(p.d) < 1e-6);
}
void avoidanceAndRotatedSweptValidation() {
  const auto ref = reference(); const auto cfg = config(); lp::FrenetPlanner planner(cfg);
  const std::vector<lp::Obstacle2d> obstacles{obstacle(5, 0, .36, .36)};
  const auto output = planner.plan(ref, input(ref, obstacles));
  CHECK(output.valid);
  double excursion = 0;
  for (const auto& p : output.path) excursion = std::max(excursion, std::abs(p.d));
  CHECK(excursion > .6);
  CHECK(!lp::pathHasCollision(output.path, obstacles, cfg.vehicle, cfg.collision_margin_m, .05, .035));
  // This fixture uses identical map/odom. The integrator must transform both
  // actual path and latest observation into continuous odom before this check.
}
void blockedRoadAndReset() {
  const auto ref = reference(); lp::FrenetPlanner planner(config());
  CHECK(planner.plan(ref, input(ref, {obstacle(5, 0, .36, .36)})).valid);
  const auto blocked = planner.plan(ref, input(ref, {obstacle(5, 0, 1, 8)}));
  CHECK(!blocked.valid); CHECK(blocked.path.empty());
  planner.reset();
  const auto clear = planner.plan(ref, input(ref)); CHECK(clear.valid);
  for (const auto& p : clear.path) CHECK(std::abs(p.d) < 1e-6);
}
void invalidConfigAndHeading() {
  auto cfg = config(); cfg.vehicle.width_m = 0;
  std::string reason; CHECK(!lp::isValid(cfg, &reason));
  const auto ref = reference(); lp::FrenetPlanner planner(config());
  auto value = input(ref); value.pose.yaw = lp::kPi;
  const auto result = planner.plan(ref, value); CHECK(!result.valid); CHECK(result.path.empty());
}
void sameStationComparison() {
  std::vector<lp::PathPoint> previous{{0, 0, 0, 0, 10, -1}, {0, 0, 0, 0, 12, 1}};
  double d = 0;
  CHECK(lp::interpolatePathLateralOffset(previous, 11, &d)); CHECK(std::abs(d) < 1e-9);
  CHECK(!lp::interpolatePathLateralOffset(previous, 9, &d));
}
void discontinuityDetection() {
  lp::PoseJumpGuard guard({}); std::string reason;
  CHECK(!guard.updatePose({0, 0, 0}, 1, &reason));
  CHECK(guard.updatePose({5, 0, 0}, 1.1, &reason));
  guard.reset();
  CHECK(!guard.updateMapToOdom({0, 0, 0}, &reason));
  CHECK(guard.updateMapToOdom({1, 0, 0}, &reason));
}
int main() {
  const std::pair<const char*, void(*)()> tests[] = {
      {"clear", clearRoad}, {"avoidance", avoidanceAndRotatedSweptValidation},
      {"blocked_reset", blockedRoadAndReset}, {"config_heading", invalidConfigAndHeading},
      {"same_station", sameStationComparison}, {"jump", discontinuityDetection}};
  for (const auto& test : tests) {
    try { test.second(); std::cout << "PASS " << test.first << '\n'; }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
  }
  std::cout << "6 scenario groups passed; no ROS or hardware started\n";
}
