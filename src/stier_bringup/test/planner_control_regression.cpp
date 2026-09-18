// Deterministic kinematic regression, not a vehicle/dynamics certification.
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "path_planner/frenet_planner.hpp"
#include "path_planner/collision_checker.hpp"
#include "path_planner/obstacle_projection.hpp"
#include "control/lateral/pure_pursuit.hpp"

namespace lp = path_planner;
namespace cp = stier_control;

bool run(double lookahead, const std::vector<lp::Waypoint>& recorded = {}) {
  lp::FrenetPlannerConfig config;
  config.vehicle = {1.35, .85, .75, .38};
  lp::ReferencePath reference;
  std::vector<lp::Waypoint> waypoints;
  if (recorded.empty()) {
    for (int i = -20; i <= 220; ++i)
      waypoints.push_back({i * .1, 0., 2., 2., 1., "synthetic"});
  } else waypoints = recorded;
  std::string error;
  if (!reference.initialize(waypoints, &error)) return false;
  lp::FrenetPlanner planner(config);
  const auto start = reference.sample(0.);
  lp::Pose2d pose = recorded.empty() ? lp::Pose2d{0.,0.,0.} :
                                      lp::Pose2d{start.x,start.y,start.yaw};
  const double initial_s = reference.projectPose(pose, 0., reference.length(), 2.).s;
  const auto cone = reference.sample(initial_s+6.);
  const double x = cone.x, y = cone.y;
  std::vector<lp::Obstacle2d> obstacles{
      {{x,y}, {{{x-.18,y-.18}, {x+.18,y-.18}, {x+.18,y+.18}, {x-.18,y+.18}}}}};
  cp::PurePursuitConfig pursuit;
  pursuit.wheelbase_m = .75;
  pursuit.lookahead_base_m = lookahead;
  pursuit.lookahead_min_m = pursuit.lookahead_max_m = lookahead;
  pursuit.minimum_target_distance_m = .2;
  pursuit.maximum_steering_angle_rad = 25. * lp::kPi / 180.;
  lp::PlannerResult path;
  double steering = 0., max_offset = 0.;
  for (int tick = 0; tick < 2400; ++tick) {
    if (tick % 4 == 0) {  // 5 Hz planner, 20 Hz control, .05 s steps
      lp::PlannerInput input;
      input.pose = pose;
      input.projection = reference.projectPose(pose, 0., reference.length(), 2.);
      input.obstacles = obstacles;
      input.frenet_obstacles = lp::projectObstaclesToFrenet(reference, obstacles,
          0., reference.length(), 5., input.projection.s, 4.);
      path = planner.plan(reference, input);
      if (!path.valid) {
        std::cout << "LD=" << lookahead << " x=" << pose.x << " y=" << pose.y
                  << " planner failure: " << path.reason << '\n';
        return false;
      }
    }
    std::vector<cp::Point2d> body;
    for (const auto& p : path.path) {
      const double dx = p.x - pose.x, dy = p.y - pose.y;
      body.push_back({std::cos(pose.yaw)*dx + std::sin(pose.yaw)*dy,
                     -std::sin(pose.yaw)*dx + std::cos(pose.yaw)*dy});
    }
    const auto command = cp::computePurePursuit(body, 1., 0., pursuit);
    if (!command.valid) return false;
    const double rate_step = .05 * lp::kPi / 4.;
    steering += std::max(-rate_step, std::min(rate_step,
                            command.steering_angle_rad - steering));
    pose.x += .05 * std::cos(pose.yaw);
    pose.y += .05 * std::sin(pose.yaw);
    pose.yaw += .05 / config.vehicle.wheelbase_m * std::tan(steering);
    const auto projection = reference.projectPose(pose, 0., reference.length(), 2.);
    max_offset = std::max(max_offset, std::abs(projection.d));
    const auto footprint = lp::vehicleFootprint(pose, config.vehicle, 0.);
    for (const auto& obstacle : obstacles) {
      if (lp::convexPolygonsIntersect(footprint, obstacle.corners)) {
        std::cout << "LD=" << lookahead << " physical collision x=" << pose.x << '\n';
        return false;
      }
    }
    if (projection.s >= (recorded.empty() ? initial_s+15. : reference.length()-.5)) {
      std::cout << "LD=" << lookahead << " completed; max_offset=" << max_offset
                << " return_error=" << projection.d << '\n';
      return std::abs(projection.d) < .2;
    }
  }
  return false;
}

int main(int argc, char** argv) {
  std::vector<lp::Waypoint> recorded;
  if (argc == 2) {
    std::ifstream csv(argv[1]);
    std::string line;
    std::getline(csv, line);
    while (std::getline(csv, line)) {
      std::stringstream row(line);
      std::vector<std::string> fields;
      std::string field;
      while (std::getline(row, field, ',')) fields.push_back(field);
      recorded.push_back({std::stod(fields.at(6)), std::stod(fields.at(7)),
                          2.,2.,1.,"recorded"});
    }
    if (recorded.size() < 2) return 2;
    std::cout << argv[1] << '\n';
  }
  const bool one = run(1., recorded);
  const bool two = run(2., recorded);
  return one && two ? 0 : 1;
}
