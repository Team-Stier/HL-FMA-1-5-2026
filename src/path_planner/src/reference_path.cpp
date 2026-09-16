#include "path_planner/reference_path.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace path_planner {
namespace {

std::string trim(const std::string& value) {
  const std::size_t first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const std::size_t last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1U);
}

std::vector<std::string> splitCsvLine(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(trim(field));
  }
  if (!line.empty() && line.back() == ',') {
    fields.emplace_back();
  }
  return fields;
}

bool finiteWaypoint(const Waypoint& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.left_bound_m) &&
         std::isfinite(point.right_bound_m) &&
         std::isfinite(point.speed_limit_kph);
}

double segmentLength(const Waypoint& first, const Waypoint& second) {
  return std::hypot(second.x - first.x, second.y - first.y);
}

double parseNumber(const std::string& text) {
  std::size_t consumed = 0U;
  const double value = std::stod(text, &consumed);
  if (consumed != text.size() || !std::isfinite(value)) {
    throw std::invalid_argument("invalid numeric CSV field");
  }
  return value;
}

}  // namespace

double normalizeAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

double interpolateAngle(double from, double to, double ratio) {
  return normalizeAngle(from + ratio * normalizeAngle(to - from));
}

double squaredDistance(const Point2d& first, const Point2d& second) {
  const double dx = first.x - second.x;
  const double dy = first.y - second.y;
  return dx * dx + dy * dy;
}

bool ReferencePath::loadCsv(const std::string& file_path, std::string* error) {
  std::ifstream stream(file_path);
  if (!stream.is_open()) {
    if (error != nullptr) {
      *error = "cannot open reference path CSV: " + file_path;
    }
    return false;
  }

  std::vector<std::string> header;
  std::unordered_map<std::string, std::size_t> columns;
  std::vector<Waypoint> waypoints;
  std::string line;
  std::size_t line_number = 0U;
  while (std::getline(stream, line)) {
    ++line_number;
    const std::string cleaned = trim(line);
    if (cleaned.empty() || cleaned.front() == '#') {
      continue;
    }
    const std::vector<std::string> fields = splitCsvLine(cleaned);
    if (header.empty()) {
      header = fields;
      for (std::size_t index = 0U; index < header.size(); ++index) {
        if (header[index].empty() || columns.count(header[index]) != 0U) {
          if (error != nullptr) {
            *error = "reference CSV has empty or duplicate column";
          }
          return false;
        }
        columns[header[index]] = index;
      }
      const char* required[] = {"x", "y", "left_bound_m", "right_bound_m"};
      for (const char* name : required) {
        if (columns.count(name) == 0U) {
          if (error != nullptr) {
            *error = "reference CSV is missing column: " + std::string(name);
          }
          return false;
        }
      }
      continue;
    }

    const auto value = [&](const std::string& name,
                           const std::string& fallback) -> std::string {
      const auto iterator = columns.find(name);
      if (iterator == columns.end() || iterator->second >= fields.size() ||
          fields[iterator->second].empty()) {
        return fallback;
      }
      return fields[iterator->second];
    };

    try {
      Waypoint point;
      point.x = parseNumber(value("x", "nan"));
      point.y = parseNumber(value("y", "nan"));
      point.left_bound_m = parseNumber(value("left_bound_m", "nan"));
      point.right_bound_m = parseNumber(value("right_bound_m", "nan"));
      point.speed_limit_kph = parseNumber(value("speed_limit_kph", "0"));
      point.mission_zone = value("mission_zone", "NONE");
      waypoints.push_back(point);
    } catch (const std::exception&) {
      if (error != nullptr) {
        *error = "invalid numeric value at reference CSV line " +
                 std::to_string(line_number);
      }
      return false;
    }
  }

  if (header.empty()) {
    if (error != nullptr) {
      *error = "reference CSV has no header";
    }
    return false;
  }
  return initialize(waypoints, error);
}

bool ReferencePath::initialize(const std::vector<Waypoint>& waypoints,
                               std::string* error) {
  if (waypoints.size() < 3U) {
    if (error != nullptr) {
      *error = "reference path requires at least three waypoints";
    }
    return false;
  }
  for (std::size_t index = 0U; index < waypoints.size(); ++index) {
    const Waypoint& point = waypoints[index];
    if (!finiteWaypoint(point) || point.left_bound_m <= 0.0 ||
        point.right_bound_m <= 0.0 || point.speed_limit_kph < 0.0) {
      if (error != nullptr) {
        *error = "invalid reference waypoint at index " +
                 std::to_string(index);
      }
      return false;
    }
    if (index > 0U && segmentLength(waypoints[index - 1U], point) < 1.0e-4) {
      if (error != nullptr) {
        *error = "duplicate consecutive reference waypoint at index " +
                 std::to_string(index);
      }
      return false;
    }
  }

  points_.clear();
  points_.resize(waypoints.size());
  double accumulated_s = 0.0;
  for (std::size_t index = 0U; index < waypoints.size(); ++index) {
    ReferencePoint& output = points_[index];
    static_cast<Waypoint&>(output) = waypoints[index];
    if (index > 0U) {
      accumulated_s += segmentLength(waypoints[index - 1U], waypoints[index]);
    }
    output.s = accumulated_s;
  }

  for (std::size_t index = 0U; index < points_.size(); ++index) {
    const std::size_t previous = (index == 0U) ? 0U : index - 1U;
    const std::size_t next =
        (index + 1U >= points_.size()) ? points_.size() - 1U : index + 1U;
    points_[index].yaw = std::atan2(points_[next].y - points_[previous].y,
                                    points_[next].x - points_[previous].x);
  }

  points_.front().curvature = 0.0;
  points_.back().curvature = 0.0;
  for (std::size_t index = 1U; index + 1U < points_.size(); ++index) {
    const ReferencePoint& first = points_[index - 1U];
    const ReferencePoint& second = points_[index];
    const ReferencePoint& third = points_[index + 1U];
    const double a = std::hypot(second.x - first.x, second.y - first.y);
    const double b = std::hypot(third.x - second.x, third.y - second.y);
    const double c = std::hypot(third.x - first.x, third.y - first.y);
    const double denominator = a * b * c;
    if (denominator > 1.0e-9) {
      const double cross = (second.x - first.x) * (third.y - first.y) -
                           (second.y - first.y) * (third.x - first.x);
      points_[index].curvature = 2.0 * cross / denominator;
    }
  }
  return true;
}

double ReferencePath::length() const {
  return points_.empty() ? 0.0 : points_.back().s;
}

ReferencePoint ReferencePath::sample(double s) const {
  if (points_.empty()) {
    return ReferencePoint{};
  }
  const double clamped_s = std::max(0.0, std::min(length(), s));
  const auto upper = std::upper_bound(
      points_.begin(), points_.end(), clamped_s,
      [](double value, const ReferencePoint& point) { return value < point.s; });
  if (upper == points_.begin()) {
    return points_.front();
  }
  if (upper == points_.end()) {
    return points_.back();
  }
  const ReferencePoint& second = *upper;
  const ReferencePoint& first = *(upper - 1);
  const double segment_s = second.s - first.s;
  const double ratio = segment_s > 1.0e-9
                           ? (clamped_s - first.s) / segment_s
                           : 0.0;
  ReferencePoint result;
  result.s = clamped_s;
  result.x = first.x + ratio * (second.x - first.x);
  result.y = first.y + ratio * (second.y - first.y);
  result.yaw = interpolateAngle(first.yaw, second.yaw, ratio);
  result.curvature =
      first.curvature + ratio * (second.curvature - first.curvature);
  result.left_bound_m =
      first.left_bound_m + ratio * (second.left_bound_m - first.left_bound_m);
  result.right_bound_m = first.right_bound_m +
                         ratio * (second.right_bound_m - first.right_bound_m);
  result.speed_limit_kph = first.speed_limit_kph +
                           ratio * (second.speed_limit_kph -
                                    first.speed_limit_kph);
  result.mission_zone = ratio < 0.5 ? first.mission_zone : second.mission_zone;
  return result;
}

Projection ReferencePath::projectPoint(const Point2d& point, double minimum_s,
                                       double maximum_s) const {
  return project(point, false, 0.0, minimum_s, maximum_s, 0.0);
}

Projection ReferencePath::projectPose(const Pose2d& pose, double minimum_s,
                                      double maximum_s,
                                      double heading_weight) const {
  return project({pose.x, pose.y}, true, pose.yaw, minimum_s, maximum_s,
                 heading_weight);
}

Projection ReferencePath::project(const Point2d& point, bool use_heading,
                                  double heading, double minimum_s,
                                  double maximum_s,
                                  double heading_weight) const {
  Projection best;
  double best_score = std::numeric_limits<double>::infinity();
  if (points_.size() < 2U || minimum_s > maximum_s ||
      !std::isfinite(point.x) || !std::isfinite(point.y) ||
      (use_heading && (!std::isfinite(heading) ||
                       !std::isfinite(heading_weight) || heading_weight < 0))) {
    return best;
  }
  if (std::isnan(minimum_s) || std::isnan(maximum_s)) {
    return best;
  }
  std::size_t begin_index = 0U;
  if (std::isfinite(minimum_s)) {
    const auto begin = std::lower_bound(
        points_.begin(), points_.end(), minimum_s,
        [](const ReferencePoint& candidate, double value) {
          return candidate.s < value;
        });
    begin_index = static_cast<std::size_t>(begin - points_.begin());
    if (begin_index > 0U) {
      --begin_index;
    }
  }
  std::size_t end_index = points_.size() - 1U;
  if (std::isfinite(maximum_s)) {
    const auto end = std::upper_bound(
        points_.begin(), points_.end(), maximum_s,
        [](double value, const ReferencePoint& candidate) {
          return value < candidate.s;
        });
    end_index = std::min<std::size_t>(
        points_.size() - 1U,
        static_cast<std::size_t>(end - points_.begin()));
  }
  for (std::size_t index = begin_index; index < end_index; ++index) {
    const ReferencePoint& first = points_[index];
    const ReferencePoint& second = points_[index + 1U];
    if (second.s < minimum_s || first.s > maximum_s) {
      continue;
    }
    const double dx = second.x - first.x;
    const double dy = second.y - first.y;
    const double length_squared = dx * dx + dy * dy;
    if (length_squared <= 1.0e-12) {
      continue;
    }
    double ratio = ((point.x - first.x) * dx + (point.y - first.y) * dy) /
                   length_squared;
    // Heading ranks distinct geometric projections (e.g. parallel branches).
    // Do not let it pull a pose backwards along the same curved branch by
    // selecting a clamped endpoint that is not a local distance minimum.
    // Such a false station produced a kink at the first Frenet sample.
    if (ratio <= 0.0 && index > 0U) {
      const ReferencePoint& previous = points_[index - 1U];
      if ((point.x - first.x) * (first.x - previous.x) +
          (point.y - first.y) * (first.y - previous.y) < 0.0) {
        continue;
      }
    }
    if (ratio >= 1.0 && index + 2U < points_.size()) {
      const ReferencePoint& next = points_[index + 2U];
      if ((point.x - second.x) * (next.x - second.x) +
          (point.y - second.y) * (next.y - second.y) > 0.0) {
        continue;
      }
    }
    ratio = std::max(0.0, std::min(1.0, ratio));
    const double projected_s = first.s + ratio * (second.s - first.s);
    if (projected_s < minimum_s || projected_s > maximum_s) {
      continue;
    }
    const double projected_x = first.x + ratio * dx;
    const double projected_y = first.y + ratio * dy;
    const double error_x = point.x - projected_x;
    const double error_y = point.y - projected_y;
    const double segment_yaw = std::atan2(dy, dx);
    const double distance_squared = error_x * error_x + error_y * error_y;
    const double heading_error = normalizeAngle(heading - segment_yaw);
    const double score = distance_squared +
                         (use_heading ? heading_weight * heading_error *
                                            heading_error
                                      : 0.0);
    if (score < best_score) {
      best_score = score;
      best.valid = true;
      best.s = projected_s;
      best.d = -std::sin(segment_yaw) * error_x +
               std::cos(segment_yaw) * error_y;
      best.distance_m = std::sqrt(distance_squared);
      best.reference_yaw = segment_yaw;
      best.segment_index = index;
    }
  }
  return best;
}

}  // namespace path_planner
