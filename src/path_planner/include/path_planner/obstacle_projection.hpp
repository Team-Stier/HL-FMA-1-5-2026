#pragma once

#include <vector>

#include "path_planner/reference_path.hpp"
#include "path_planner/types.hpp"

namespace path_planner {

// Projects only onto the active route station window. Obstacle corners are
// expressed in the center projection's tangent frame so a corner cannot jump
// to a nearby parallel or returning reference segment.
std::vector<FrenetObstacle> projectObstaclesToFrenet(
    const ReferencePath& reference, const std::vector<Obstacle2d>& obstacles,
    double minimum_s, double maximum_s, double maximum_distance_m,
    double anchor_s, double station_tolerance_m);

}  // namespace path_planner
