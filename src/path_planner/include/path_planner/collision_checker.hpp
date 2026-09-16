#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include "path_planner/types.hpp"

namespace path_planner {

using Polygon4 = std::array<Point2d, 4U>;

// Builds the body rectangle around a rear-axle pose. The safety margin is
// applied on all four sides of the vehicle.
Polygon4 vehicleFootprint(const Pose2d& rear_axle_pose,
                          const VehicleGeometry& vehicle,
                          double margin_m);

bool convexPolygonsIntersect(const Polygon4& first, const Polygon4& second);

bool vehicleIntersectsObstacle(const Pose2d& rear_axle_pose,
                               const VehicleGeometry& vehicle,
                               double margin_m,
                               const Obstacle2d& obstacle);

bool pathHasCollision(const std::vector<PathPoint>& path,
                      const std::vector<Obstacle2d>& obstacles,
                      const VehicleGeometry& vehicle, double margin_m,
                      double maximum_translation_step_m,
                      double maximum_yaw_step_rad,
                      std::size_t* collision_index = nullptr);

}  // namespace path_planner
