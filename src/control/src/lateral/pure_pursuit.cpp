#include "control/lateral/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace stier_control {
namespace {

bool isFinitePoint(const Point2d& point) {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

void requireFiniteNonNegative(const char* name, double value) {
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(std::string(name) +
                                " must be finite and non-negative");
  }
}

void requireFinitePositive(const char* name, double value) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) +
                                " must be finite and positive");
  }
}

void validateConfig(const PurePursuitConfig& config) {
  requireFinitePositive("wheelbase_m", config.wheelbase_m);
  requireFiniteNonNegative("lookahead_base_m", config.lookahead_base_m);
  requireFiniteNonNegative("lookahead_speed_gain_sec",
                           config.lookahead_speed_gain_sec);
  requireFiniteNonNegative("lookahead_curvature_gain_m",
                           config.lookahead_curvature_gain_m);
  requireFinitePositive("lookahead_min_m", config.lookahead_min_m);
  requireFinitePositive("lookahead_max_m", config.lookahead_max_m);
  if (config.lookahead_max_m < config.lookahead_min_m) {
    throw std::invalid_argument(
        "lookahead_max_m must be at least lookahead_min_m");
  }
  requireFinitePositive("minimum_target_distance_m",
                        config.minimum_target_distance_m);
  requireFinitePositive("maximum_steering_angle_rad",
                        config.maximum_steering_angle_rad);
}

bool firstForwardIntersection(const Point2d& start, const Point2d& end,
                              double radius, Point2d* target) {
  if (!isFinitePoint(start) || !isFinitePoint(end) || target == nullptr) {
    return false;
  }

  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const double a = dx * dx + dy * dy;
  if (a == 0.0) {
    return false;
  }

  const double b = 2.0 * (start.x * dx + start.y * dy);
  const double c = start.x * start.x + start.y * start.y - radius * radius;
  const double discriminant = b * b - 4.0 * a * c;
  if (discriminant < 0.0 || !std::isfinite(discriminant)) {
    return false;
  }

  const double root = std::sqrt(discriminant);
  const double roots[] = {
      (-b - root) / (2.0 * a),
      (-b + root) / (2.0 * a),
  };
  for (const double t : roots) {
    if (t < 0.0 || t > 1.0 || !std::isfinite(t)) {
      continue;
    }
    const Point2d candidate{start.x + t * dx, start.y + t * dy};
    if (candidate.x > 0.0 && isFinitePoint(candidate)) {
      *target = candidate;
      return true;
    }
  }
  return false;
}

}  // namespace

PurePursuitResult computePurePursuit(
    const std::vector<Point2d>& path_in_rear_axle_frame,
    double longitudinal_speed_mps, double preview_curvature_m_inv,
    const PurePursuitConfig& config) {
  validateConfig(config);
  if (!std::isfinite(longitudinal_speed_mps) ||
      !std::isfinite(preview_curvature_m_inv) ||
      preview_curvature_m_inv < 0.0) {
    return {};
  }

  PurePursuitResult result;
  const double speed_lookahead_m =
      config.lookahead_base_m +
      config.lookahead_speed_gain_sec * std::abs(longitudinal_speed_mps);
  const double curvature_scale =
      1.0 + config.lookahead_curvature_gain_m * preview_curvature_m_inv;
  result.lookahead_m = std::max(
      config.lookahead_min_m,
      std::min(config.lookahead_max_m, speed_lookahead_m / curvature_scale));

  Point2d target;
  double nearest_distance = std::numeric_limits<double>::infinity();
  std::size_t next_index = path_in_rear_axle_frame.size();
  Point2d projection;
  for (std::size_t i = 1; i < path_in_rear_axle_frame.size(); ++i) {
    const auto& a = path_in_rear_axle_frame[i - 1];
    const auto& b = path_in_rear_axle_frame[i];
    if (!isFinitePoint(a) || !isFinitePoint(b)) continue;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length_squared = dx * dx + dy * dy;
    if (length_squared <= 1e-12) continue;
    const double t = std::max(
        0.0, std::min(1.0, -(a.x * dx + a.y * dy) / length_squared));
    const Point2d point{a.x + t * dx, a.y + t * dy};
    const double distance = std::hypot(point.x, point.y);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      projection = point;
      next_index = i;
    }
  }
  if (!std::isfinite(nearest_distance)) return result;

  bool found_target = false;
  for (std::size_t index = next_index; index < path_in_rear_axle_frame.size();
       ++index) {
    if (firstForwardIntersection(index == next_index
                                     ? projection
                                     : path_in_rear_axle_frame[index - 1],
                                 path_in_rear_axle_frame[index],
                                 result.lookahead_m, &target)) {
      found_target = true;
      break;
    }
  }

  if (!found_target) {
    // Project onto the nearest segment, then advance along the path instead
    // of aiming at its far end when the lookahead circle cannot reach it.
    target = projection;
    double remaining = result.lookahead_m;
    for (std::size_t i = next_index; i < path_in_rear_axle_frame.size(); ++i) {
      const auto& point = path_in_rear_axle_frame[i];
      if (!isFinitePoint(point)) return result;
      const double distance = std::hypot(point.x - target.x, point.y - target.y);
      if (distance > remaining) {
        const double t = remaining / distance;
        target = {target.x + t * (point.x - target.x),
                  target.y + t * (point.y - target.y)};
        break;
      }
      target = point;
      remaining -= distance;
      if (remaining <= 1e-9) break;
    }
    if (target.x <= 0.0 ||
        std::hypot(target.x, target.y) < config.minimum_target_distance_m) {
      return result;
    }
  }

  const double distance_squared = target.x * target.x + target.y * target.y;
  const double raw =
      std::atan2(2.0 * config.wheelbase_m * target.y, distance_squared);
  result.valid = true;
  result.target = target;
  result.steering_angle_rad = std::max(
      -config.maximum_steering_angle_rad,
      std::min(config.maximum_steering_angle_rad, raw));
  return result;
}

}  // namespace stier_control
