#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "path_planner/types.hpp"

namespace path_planner {

class ReferencePath {
 public:
  bool loadCsv(const std::string& file_path, std::string* error);
  bool initialize(const std::vector<Waypoint>& waypoints, std::string* error);

  bool empty() const { return points_.empty(); }
  double length() const;
  const std::vector<ReferencePoint>& points() const { return points_; }

  ReferencePoint sample(double s) const;
  Projection projectPoint(
      const Point2d& point,
      double minimum_s = -std::numeric_limits<double>::infinity(),
      double maximum_s = std::numeric_limits<double>::infinity()) const;
  Projection projectPose(
      const Pose2d& pose, double minimum_s,
      double maximum_s, double heading_weight) const;

 private:
  Projection project(const Point2d& point, bool use_heading, double heading,
                     double minimum_s, double maximum_s,
                     double heading_weight) const;

  std::vector<ReferencePoint> points_;
};

}  // namespace path_planner
