#include "lidar_path_planning/frenet_planner.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "lidar_path_planning/boundary_checker.hpp"
#include "lidar_path_planning/collision_checker.hpp"

namespace lidar_path_planning {
namespace {

struct QuinticPolynomial {
  double c0{0.0};
  double c1{0.0};
  double c2{0.0};
  double c3{0.0};
  double c4{0.0};
  double c5{0.0};

  QuinticPolynomial(double position_0, double velocity_0,
                    double acceleration_0, double position_1,
                    double velocity_1, double acceleration_1,
                    double length) {
    c0 = position_0;
    c1 = velocity_0;
    c2 = acceleration_0 / 2.0;
    const double length2 = length * length;
    const double length3 = length2 * length;
    const double length4 = length3 * length;
    const double length5 = length4 * length;
    c3 = (20.0 * (position_1 - position_0) -
          (8.0 * velocity_1 + 12.0 * velocity_0) * length -
          (3.0 * acceleration_0 - acceleration_1) * length2) /
         (2.0 * length3);
    c4 = (30.0 * (position_0 - position_1) +
          (14.0 * velocity_1 + 16.0 * velocity_0) * length +
          (3.0 * acceleration_0 - 2.0 * acceleration_1) * length2) /
         (2.0 * length4);
    c5 = (12.0 * (position_1 - position_0) -
          (6.0 * velocity_1 + 6.0 * velocity_0) * length -
          (acceleration_0 - acceleration_1) * length2) /
         (2.0 * length5);
  }

  double position(double value) const {
    return c0 + value *
                    (c1 + value *
                              (c2 + value *
                                        (c3 + value * (c4 + value * c5))));
  }

  double first(double value) const {
    return c1 + value *
                    (2.0 * c2 + value *
                                    (3.0 * c3 + value *
                                                    (4.0 * c4 + value *
                                                                    5.0 * c5)));
  }

  double second(double value) const {
    return 2.0 * c2 + value *
                          (6.0 * c3 + value *
                                          (12.0 * c4 + value * 20.0 * c5));
  }

  double third(double value) const {
    return 6.0 * c3 + value * (24.0 * c4 + value * 60.0 * c5);
  }
};

bool positiveFinite(double value) {
  return std::isfinite(value) && value > 0.0;
}

double pointCurvature(const PathPoint& first, const PathPoint& second,
                      const PathPoint& third) {
  const double a = std::hypot(second.x - first.x, second.y - first.y);
  const double b = std::hypot(third.x - second.x, third.y - second.y);
  const double c = std::hypot(third.x - first.x, third.y - first.y);
  const double denominator = a * b * c;
  if (denominator <= 1.0e-9) {
    return 0.0;
  }
  const double cross = (second.x - first.x) * (third.y - first.y) -
                       (second.y - first.y) * (third.x - first.x);
  return 2.0 * cross / denominator;
}

double intervalDistance(double value, double minimum, double maximum) {
  if (value < minimum) {
    return minimum - value;
  }
  if (value > maximum) {
    return value - maximum;
  }
  return 0.0;
}

std::vector<double> lateralSamples(double maximum, double interval) {
  std::vector<double> samples;
  const int count = static_cast<int>(std::floor(maximum / interval + 1.0e-9));
  samples.reserve(static_cast<std::size_t>(2 * count + 3));
  for (int index = -count; index <= count; ++index) {
    samples.push_back(static_cast<double>(index) * interval);
  }
  samples.push_back(-maximum);
  samples.push_back(0.0);
  samples.push_back(maximum);
  std::sort(samples.begin(), samples.end());
  samples.erase(std::unique(samples.begin(), samples.end(),
                            [](double first, double second) {
                              return std::abs(first - second) < 1.0e-9;
                            }),
                samples.end());
  // Evaluate the nominal route and small deviations first. This normally
  // establishes a low-cost, precisely boundary-checked incumbent immediately.
  std::sort(samples.begin(), samples.end(), [](double first, double second) {
    const double first_magnitude = std::abs(first);
    const double second_magnitude = std::abs(second);
    if (std::abs(first_magnitude - second_magnitude) > 1.0e-9) {
      return first_magnitude < second_magnitude;
    }
    return first < second;
  });
  return samples;
}

bool pathWithinReferenceBounds(const ReferencePath& reference,
                               const std::vector<PathPoint>& path,
                               const FrenetPlannerConfig& config) {
  return sweptPathWithinReferenceBounds(
      reference, path, config.vehicle, config.boundary_margin_m,
      config.boundary_perimeter_sample_interval_m, 0.05, 2.0 * kPi / 180.0);
}

}  // namespace

struct FrenetPlanner::Candidate {
  bool feasible{true};
  double cost{0.0};
  std::vector<PathPoint> path;
};

bool interpolatePathLateralOffset(const std::vector<PathPoint>& path,
                                  double route_s, double* lateral_offset) {
  if (path.empty() || lateral_offset == nullptr ||
      route_s < path.front().s - 1.0e-9 ||
      route_s > path.back().s + 1.0e-9) {
    return false;
  }
  const auto upper = std::lower_bound(
      path.begin(), path.end(), route_s,
      [](const PathPoint& point, double value) { return point.s < value; });
  if (upper == path.begin()) {
    *lateral_offset = upper->d;
    return true;
  }
  if (upper == path.end()) {
    *lateral_offset = path.back().d;
    return true;
  }
  const PathPoint& second = *upper;
  const PathPoint& first = *(upper - 1);
  const double station_interval = second.s - first.s;
  if (station_interval <= 1.0e-9) {
    *lateral_offset = second.d;
    return true;
  }
  const double ratio = (route_s - first.s) / station_interval;
  *lateral_offset = first.d + ratio * (second.d - first.d);
  return true;
}

bool isValid(const FrenetPlannerConfig& config, std::string* reason) {
  const bool vehicle_valid = positiveFinite(config.vehicle.length_m) &&
                             positiveFinite(config.vehicle.width_m) &&
                             positiveFinite(config.vehicle.wheelbase_m) &&
                             std::isfinite(
                                 config.vehicle.rear_axle_to_center_m) &&
                             config.vehicle.rear_axle_to_center_m >= 0.0 &&
                             config.vehicle.rear_axle_to_center_m <=
                                 config.vehicle.length_m / 2.0 &&
                             config.vehicle.wheelbase_m <=
                                 config.vehicle.rear_axle_to_center_m +
                                     config.vehicle.length_m / 2.0;
  const bool midpoints_valid =
      !config.midpoint_fractions.empty() &&
      std::all_of(config.midpoint_fractions.begin(),
                  config.midpoint_fractions.end(), [](double fraction) {
                    return std::isfinite(fraction) && fraction > 0.2 &&
                           fraction < 0.8;
                  });
  const bool sampling_valid = positiveFinite(config.horizon_m) &&
                              positiveFinite(config.sample_interval_m) &&
                              positiveFinite(config.lateral_sample_interval_m) &&
                              positiveFinite(config.maximum_lateral_offset_m) &&
                              midpoints_valid;
  const bool limits_valid = config.collision_margin_m >= 0.0 &&
                            config.boundary_margin_m >= 0.0 &&
                            positiveFinite(config.maximum_curvature_per_m) &&
                            positiveFinite(config.maximum_curvature_rate_per_m2) &&
                            positiveFinite(config.maximum_lateral_slope) &&
                            positiveFinite(config.minimum_longitudinal_scale) &&
                            config.minimum_longitudinal_scale < 1.0 &&
                            positiveFinite(
                                config.boundary_perimeter_sample_interval_m);
  const double weights[] = {
      config.weight_deviation,       config.weight_smoothness,
      config.weight_jerk,            config.weight_clearance,
      config.weight_terminal_offset, config.weight_previous_path,
  };
  const bool weights_valid =
      std::all_of(std::begin(weights), std::end(weights), [](double weight) {
        return std::isfinite(weight) && weight >= 0.0;
      });
  if (!vehicle_valid || !sampling_valid || !limits_valid || !weights_valid) {
    if (reason != nullptr) {
      *reason = "invalid Frenet vehicle geometry, sampling, or limit parameter";
    }
    return false;
  }
  return true;
}

FrenetPlanner::FrenetPlanner(const FrenetPlannerConfig& config)
    : config_(config) {}

PlannerResult FrenetPlanner::plan(const ReferencePath& reference,
                                  const PlannerInput& input) {
  PlannerResult result;
  std::string config_error;
  if (!isValid(config_, &config_error)) {
    result.reason = config_error;
    return result;
  }
  if (!input.projection.valid) {
    result.reason = "vehicle pose is not projected onto reference path";
    return result;
  }

  const double start_s = input.projection.s;
  const double end_s = std::min(reference.length(), start_s + config_.horizon_m);
  const double available_horizon = end_s - start_s;
  if (available_horizon < std::max(1.0, 4.0 * config_.sample_interval_m)) {
    result.reason = "insufficient reference path ahead";
    return result;
  }
  const ReferencePoint start_reference = reference.sample(start_s);
  const double start_longitudinal_scale =
      1.0 - start_reference.curvature * input.projection.d;
  if (!std::isfinite(start_longitudinal_scale) ||
      start_longitudinal_scale < config_.minimum_longitudinal_scale) {
    result.reason = "Frenet longitudinal scale is singular at vehicle pose";
    return result;
  }
  const double heading_error =
      normalizeAngle(input.pose.yaw - start_reference.yaw);
  // tan(yaw_error) alone aliases a backward-facing pose to a forward pose.
  // Reverse parking requires its own direction-aware planner and controller.
  if (!std::isfinite(heading_error) || std::abs(heading_error) >= kPi / 2.0) {
    result.reason = "vehicle heading is opposite to forward reference";
    return result;
  }
  // For x(s, d)=reference(s)+normal(s)*d, the exact relation is
  // d'=(1-kappa*d)*tan(yaw_error). Omitting the scale creates a yaw jump at
  // the first candidate point on curved references.
  const double start_slope =
      start_longitudinal_scale * std::tan(heading_error);
  if (!std::isfinite(start_slope) ||
      std::abs(start_slope) > config_.maximum_lateral_slope) {
    result.reason = "vehicle heading is too far from reference path";
    return result;
  }

  const std::vector<double> offsets =
      lateralSamples(config_.maximum_lateral_offset_m,
                     config_.lateral_sample_interval_m);
  std::vector<Candidate> ranked_candidates;
  ranked_candidates.reserve(config_.midpoint_fractions.size() *
                             offsets.size() * offsets.size());

  for (const double midpoint_fraction : config_.midpoint_fractions) {
    const double middle_s = start_s + midpoint_fraction * available_horizon;
    const double first_length = middle_s - start_s;
    const double second_length = end_s - middle_s;
    for (const double middle_d : offsets) {
      for (const double end_d : offsets) {
        ++result.evaluated_candidates;
        Candidate candidate;
        const QuinticPolynomial first(input.projection.d, start_slope, 0.0,
                                      middle_d, 0.0, 0.0, first_length);
        const QuinticPolynomial second(middle_d, 0.0, 0.0, end_d, 0.0, 0.0,
                                       second_length);
        const std::size_t sample_count = static_cast<std::size_t>(
            std::ceil(available_horizon / config_.sample_interval_m));
        candidate.path.reserve(sample_count + 1U);

        for (std::size_t sample_index = 0U; sample_index <= sample_count;
             ++sample_index) {
          const double route_s =
              std::min(end_s, start_s +
                                  static_cast<double>(sample_index) *
                                      config_.sample_interval_m);
          const bool first_half = route_s <= middle_s;
          const double polynomial_s =
              first_half ? route_s - start_s : route_s - middle_s;
          const QuinticPolynomial& polynomial = first_half ? first : second;
          const double d = polynomial.position(polynomial_s);
          const double slope = polynomial.first(polynomial_s);
          const double second_derivative = polynomial.second(polynomial_s);
          const double third_derivative = polynomial.third(polynomial_s);
          if (!std::isfinite(d) || !std::isfinite(slope) ||
              !std::isfinite(second_derivative) ||
              !std::isfinite(third_derivative) ||
              std::abs(slope) > config_.maximum_lateral_slope) {
            candidate.feasible = false;
            break;
          }

          const ReferencePoint reference_point = reference.sample(route_s);
          PathPoint point;
          point.s = route_s;
          point.d = d;
          point.x = reference_point.x - std::sin(reference_point.yaw) * d;
          point.y = reference_point.y + std::cos(reference_point.yaw) * d;
          const double longitudinal_scale =
              1.0 - reference_point.curvature * d;
          if (!std::isfinite(longitudinal_scale) ||
              longitudinal_scale < config_.minimum_longitudinal_scale) {
            candidate.feasible = false;
            break;
          }
          point.yaw = normalizeAngle(
              reference_point.yaw +
              std::atan2(slope, longitudinal_scale));
          if (sample_index == 0U) {
            // The polyline projection tangent and the smoothed reference yaw
            // can differ slightly at a waypoint. Always anchor the published
            // candidate to the measured rear-axle pose exactly.
            point.x = input.pose.x;
            point.y = input.pose.y;
            point.yaw = input.pose.yaw;
          }

          const double relative_yaw =
              normalizeAngle(point.yaw - reference_point.yaw);
          const double body_center_d =
              d + config_.vehicle.rear_axle_to_center_m *
                      std::sin(relative_yaw);
          const double lateral_extent =
              config_.vehicle.length_m / 2.0 *
                  std::abs(std::sin(relative_yaw)) +
              config_.vehicle.width_m / 2.0 *
                  std::abs(std::cos(relative_yaw));
          if (body_center_d + lateral_extent + config_.boundary_margin_m >
                  reference_point.left_bound_m ||
              body_center_d - lateral_extent - config_.boundary_margin_m <
                  -reference_point.right_bound_m) {
            candidate.feasible = false;
            break;
          }

          const Pose2d rear_axle_pose{point.x, point.y, point.yaw};
          const Polygon4 footprint = vehicleFootprint(
              rear_axle_pose, config_.vehicle, config_.collision_margin_m);
          for (const Obstacle2d& obstacle : input.obstacles) {
            if (convexPolygonsIntersect(footprint, obstacle.corners)) {
              candidate.feasible = false;
              break;
            }
          }
          if (!candidate.feasible) {
            break;
          }

          const double vehicle_center_s =
              route_s + config_.vehicle.rear_axle_to_center_m *
                            std::cos(relative_yaw);
          double minimum_clearance = std::numeric_limits<double>::infinity();
          for (const FrenetObstacle& obstacle : input.frenet_obstacles) {
            const double expanded_s_min =
                obstacle.s_min - config_.vehicle.length_m / 2.0 -
                config_.collision_margin_m;
            const double expanded_s_max =
                obstacle.s_max + config_.vehicle.length_m / 2.0 +
                config_.collision_margin_m;
            const double expanded_d_min =
                obstacle.d_min - config_.vehicle.width_m / 2.0 -
                config_.collision_margin_m;
            const double expanded_d_max =
                obstacle.d_max + config_.vehicle.width_m / 2.0 +
                config_.collision_margin_m;
            const double s_clearance = intervalDistance(
                vehicle_center_s, expanded_s_min, expanded_s_max);
            const double d_clearance = intervalDistance(
                body_center_d, expanded_d_min, expanded_d_max);
            minimum_clearance = std::min(
                minimum_clearance, std::hypot(s_clearance, d_clearance));
          }

          candidate.path.push_back(point);
          const double integration_step = config_.sample_interval_m;
          candidate.cost +=
              integration_step *
              (config_.weight_deviation * d * d +
               config_.weight_smoothness * second_derivative *
                   second_derivative +
               config_.weight_jerk * third_derivative * third_derivative);
          if (std::isfinite(minimum_clearance)) {
            candidate.cost +=
                integration_step * config_.weight_clearance /
                ((minimum_clearance + 0.05) * (minimum_clearance + 0.05));
          }
        }

        if (!candidate.feasible || candidate.path.size() < 3U) {
          continue;
        }

        for (std::size_t index = 1U; index + 1U < candidate.path.size();
             ++index) {
          candidate.path[index].curvature = pointCurvature(
              candidate.path[index - 1U], candidate.path[index],
              candidate.path[index + 1U]);
          if (std::abs(candidate.path[index].curvature) >
              config_.maximum_curvature_per_m) {
            candidate.feasible = false;
            break;
          }
          if (index > 1U) {
            const double distance = std::hypot(
                candidate.path[index].x - candidate.path[index - 1U].x,
                candidate.path[index].y - candidate.path[index - 1U].y);
            const double curvature_rate =
                distance > 1.0e-6
                    ? std::abs(candidate.path[index].curvature -
                               candidate.path[index - 1U].curvature) /
                          distance
                    : std::numeric_limits<double>::infinity();
            if (curvature_rate > config_.maximum_curvature_rate_per_m2) {
              candidate.feasible = false;
              break;
            }
          }
        }
        if (!candidate.feasible) {
          continue;
        }
        candidate.path.front().curvature = candidate.path[1U].curvature;
        candidate.path.back().curvature =
            candidate.path[candidate.path.size() - 2U].curvature;
        candidate.cost += config_.weight_terminal_offset * end_d * end_d;

        double previous_path_cost = 0.0;
        std::size_t previous_path_samples = 0U;
        for (const PathPoint& point : candidate.path) {
          double previous_d = 0.0;
          if (interpolatePathLateralOffset(previous_path_, point.s,
                                           &previous_d)) {
            const double difference = point.d - previous_d;
            previous_path_cost += difference * difference;
            ++previous_path_samples;
          }
        }
        if (previous_path_samples > 0U) {
          candidate.cost +=
              config_.weight_previous_path * previous_path_cost /
              static_cast<double>(previous_path_samples);
        }

        ranked_candidates.push_back(std::move(candidate));
        ++result.feasible_candidates;
      }
    }
  }

  // Run expensive swept-body checks in cost order. Testing every improving
  // incumbent multiplies work on dense S-course references. The selected path
  // still has the minimum cost among candidates passing the same hard checks.
  std::stable_sort(ranked_candidates.begin(), ranked_candidates.end(),
                   [](const Candidate& a, const Candidate& b) {
                     return a.cost < b.cost;
                   });
  for (const Candidate& candidate : ranked_candidates) {
    if (!pathWithinReferenceBounds(reference, candidate.path, config_) ||
        pathHasCollision(candidate.path, input.obstacles, config_.vehicle,
                         config_.collision_margin_m, 0.05, 2.0 * kPi / 180.0)) {
      continue;
    }
    previous_path_ = candidate.path;
    result.valid = true;
    result.path = candidate.path;
    result.reason = "OK";
    return result;
  }
  result.reason = "no collision-free Frenet candidate";
  return result;
}

}  // namespace lidar_path_planning
