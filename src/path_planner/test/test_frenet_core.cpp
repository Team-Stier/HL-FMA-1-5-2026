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
  CHECK(output.reason == "RDDF_CLEAR"); CHECK(output.evaluated_candidates == 0U);
  for (const auto& p : output.path) CHECK(std::abs(p.d) < 1e-6);
  const auto roadside = planner.plan(ref, input(ref, {obstacle(4, 2, .4, .4)}));
  CHECK(roadside.valid); CHECK(roadside.reason == "RDDF_CLEAR");
  CHECK(roadside.evaluated_candidates == 0U);
  auto offset = input(ref);
  offset.pose = {0, .8, 1.0};
  offset.projection = ref.projectPose(offset.pose, 0, 10, 2);
  const auto recovery = planner.plan(ref, offset);
  CHECK(recovery.valid); CHECK(recovery.reason == "RDDF_CLEAR");
  for (const auto& p : recovery.path) CHECK(std::abs(p.y) < 1e-6);
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
void clearCurvedReferenceKeepsVertices() {
  lp::ReferencePath ref;
  std::string error;
  CHECK(ref.initialize({{0,0,1.7,1.7,5,"S"}, {1,.4,1.7,1.7,5,"S"},
                        {2,-.4,1.7,1.7,5,"S"}, {3,0,1.7,1.7,5,"S"},
                        {10,0,1.7,1.7,5,"S"}}, &error));
  auto cfg = config(); cfg.maximum_curvature_per_m = .01;
  lp::FrenetPlanner planner(cfg);
  const auto result = planner.plan(ref, input(ref));
  CHECK(result.valid); CHECK(result.evaluated_candidates == 0U);
  for (const auto& vertex : ref.points()) {
    if (vertex.s > result.path.back().s) continue;
    CHECK(std::any_of(result.path.begin(), result.path.end(), [&](const lp::PathPoint& p) {
      return std::hypot(p.x-vertex.x, p.y-vertex.y) < 1e-8;
    }));
  }
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
void hongikObstacleRegression() {
  // Recorded S reference and vehicle/cluster snapshot. Previously all 507
  // candidates failed with the unchanged curvature/rate limits.
  lp::ReferencePath ref; std::string error;
  CHECK(ref.initialize({
      {-10.940064, 38.013119, 1.7, 1.7, 1., "S"},
      {-11.346031, 38.200873, 1.7, 1.7, 1., "S"},
      {-11.751999, 38.388626, 1.7, 1.7, 1., "S"},
      {-12.199371, 38.610165, 1.7, 1.7, 1., "S"},
      {-12.646745, 38.831703, 1.7, 1.7, 1., "S"},
      {-13.026042, 38.987069, 1.7, 1.7, 1., "S"},
      {-13.405338, 39.142435, 1.7, 1.7, 1., "S"},
      {-13.830245, 39.287188, 1.7, 1.7, 1., "S"},
      {-14.255152, 39.431940, 1.7, 1.7, 1., "S"},
      {-14.618651, 39.518617, 1.7, 1.7, 1., "S"},
      {-14.982150, 39.605295, 1.7, 1.7, 1., "S"},
      {-15.400726, 39.659860, 1.7, 1.7, 1., "S"},
      {-15.819306, 39.714423, 1.7, 1.7, 1., "S"},
      {-16.177876, 39.696113, 1.7, 1.7, 1., "S"},
      {-16.536446, 39.677803, 1.7, 1.7, 1., "S"},
      {-16.853611, 39.625709, 1.7, 1.7, 1., "S"},
      {-17.170776, 39.573614, 1.7, 1.7, 1., "S"},
      {-17.520215, 39.494714, 1.7, 1.7, 1., "S"},
      {-17.869655, 39.415812, 1.7, 1.7, 1., "S"},
      {-18.222597, 39.299914, 1.7, 1.7, 1., "S"},
      {-18.575540, 39.184015, 1.7, 1.7, 1., "S"},
      {-18.976893, 39.027905, 1.7, 1.7, 1., "S"},
      {-19.378248, 38.871796, 1.7, 1.7, 1., "S"},
      {-19.706622, 38.701310, 1.7, 1.7, 1., "S"},
      {-20.034996, 38.530826, 1.7, 1.7, 1., "S"},
  }, &error));
  auto cfg = config(); cfg.vehicle = {1.35, .85, .75, .38};
  lp::PlannerInput value;
  value.pose = {-10.914117742934248, 38.13568692630359,
                2*std::atan2(.973689488668434, .22787887057953304)};
  value.projection = ref.projectPose(value.pose, 0, 15, 2);
  value.obstacles = {obstacle(-15.2024, 40.37065, .286, .4617)};
  value.frenet_obstacles = lp::projectObstaclesToFrenet(
      ref, value.obstacles, 0, 15, 5, value.projection.s, 4);
  lp::FrenetPlanner planner(cfg);
  const auto result = planner.plan(ref, value);
  CHECK(result.valid); CHECK(result.reason == "OK");
  CHECK(!lp::pathHasCollision(result.path, value.obstacles,
       cfg.vehicle, cfg.collision_margin_m, .05, .035));
  CHECK(lp::sweptPathWithinReferenceBounds(ref, result.path, cfg.vehicle,
       cfg.boundary_margin_m, .1, .05, .035));
  CHECK(std::hypot(result.path.front().x-value.pose.x,
                   result.path.front().y-value.pose.y) < 1e-9);
  for (std::size_t i=1; i+1<result.path.size(); ++i) {
    CHECK(std::abs(result.path[i].curvature) <= cfg.maximum_curvature_per_m);
    if(i>1) {
      const auto& a=result.path[i-1]; const auto& b=result.path[i];
      CHECK(std::abs(b.curvature-a.curvature)/std::hypot(b.x-a.x,b.y-a.y)
            <= cfg.maximum_curvature_rate_per_m2);
    }
  }
  // A fully blocked corridor still must not yield a drivable path.
  value.obstacles = {obstacle(-15.2, 40., 8., 8.)};
  const auto blocked = planner.plan(ref, value);
  CHECK(!blocked.valid); CHECK(blocked.path.empty());
}
int main() {
  const std::pair<const char*, void(*)()> tests[] = {
      {"clear", clearRoad}, {"avoidance", avoidanceAndRotatedSweptValidation},
      {"clear_curved_reference", clearCurvedReferenceKeepsVertices},
      {"hongik_obstacle_regression", hongikObstacleRegression},
      {"blocked_reset", blockedRoadAndReset}, {"config_heading", invalidConfigAndHeading},
      {"same_station", sameStationComparison}, {"jump", discontinuityDetection}};
  for (const auto& test : tests) {
    try { test.second(); std::cout << "PASS " << test.first << '\n'; }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
  }
  std::cout << "8 scenario groups passed; no ROS or hardware started\n";
}
