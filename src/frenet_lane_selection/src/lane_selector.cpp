#include "frenet_lane_selection/lane_selector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace frenet_lane_selection {
namespace {
constexpr double kPi = 3.14159265358979323846;
bool positive(double value) { return std::isfinite(value) && value > 0.0; }
bool nonnegative(double value) { return std::isfinite(value) && value >= 0.0; }
bool knownEnum(Lane lane) {
  return lane == Lane::Unknown || lane == Lane::Left || lane == Lane::Right;
}
Result invalid(const std::string& reason) {
  Result result;
  result.reason = reason;
  return result;
}
double yawDifference(double a, double b) {
  return std::remainder(a - b, 2.0 * kPi);
}
struct Cluster { double low; double high; };

std::vector<Cluster> clusters(std::vector<double> points, const Config& config) {
  std::sort(points.begin(), points.end());
  std::vector<Cluster> output;
  std::size_t begin = 0;
  while (begin < points.size()) {
    std::size_t end = begin + 1;
    while (end < points.size() && points[end] - points[end - 1] <= config.cluster_gap_m) {
      ++end;
    }
    if (end - begin >= config.minimum_cluster_points &&
        points[end - 1] - points[begin] <= config.maximum_cluster_span_m) {
      output.push_back({points[begin], points[end - 1]});
    }
    begin = end;
  }
  return output;
}
}  // namespace

bool validConfig(const Config& c) {
  return positive(c.expected_road_width_m) && nonnegative(c.road_width_tolerance_m) &&
      c.road_width_tolerance_m < c.expected_road_width_m / 4.0 &&
      positive(c.cluster_gap_m) && positive(c.maximum_cluster_span_m) &&
      c.cluster_gap_m <= c.maximum_cluster_span_m && c.minimum_cluster_points >= 2 &&
      c.minimum_cluster_points <= c.maximum_points_per_section &&
      c.minimum_sections >= 3 && c.minimum_sections <= c.maximum_sections &&
      c.maximum_sections <= 256 && c.maximum_points_per_section <= 4096 &&
      c.maximum_clusters_per_section >= 2 && c.maximum_clusters_per_section <= 64 &&
      positive(c.minimum_visible_length_m) && positive(c.maximum_section_gap_m) &&
      positive(c.maximum_boundary_slope) && nonnegative(c.boundary_jitter_m) &&
      positive(c.maximum_reference_yaw_rate_per_m) &&
      nonnegative(c.maximum_reference_chord_error_m) &&
      positive(c.maximum_absolute_offset_m) && positive(c.vehicle_width_m) &&
      nonnegative(c.lateral_margin_m) &&
      (c.expected_road_width_m - c.road_width_tolerance_m) / 2.0 >
          c.vehicle_width_m + 2.0 * c.lateral_margin_m &&
      positive(c.maximum_observation_age_sec) && positive(c.maximum_permission_age_sec) &&
      nonnegative(c.lane_tie_tolerance_m);
}

Result selectLane(const Observation& observation, const Permission& permission,
                  const SelectionContext& context, const Config& config) {
  if (!validConfig(config)) return invalid("INVALID_CONFIG");
  if (!observation.input_contract_valid) return invalid("INPUT_CONTRACT_INVALID");
  if (!nonnegative(observation.age_sec) ||
      observation.age_sec > config.maximum_observation_age_sec) return invalid("OBSERVATION_STALE");
  if (!permission.valid || !nonnegative(permission.age_sec) ||
      permission.age_sec > config.maximum_permission_age_sec) return invalid("PERMISSION_INVALID_OR_STALE");
  if (!knownEnum(permission.requested) || !knownEnum(context.current_lane) ||
      !std::isfinite(context.current_offset_m) ||
      std::abs(context.current_offset_m) > config.maximum_absolute_offset_m) return invalid("CONTEXT_INVALID");
  if (!permission.left_allowed && !permission.right_allowed) return invalid("NO_ALLOWED_LANE");
  if ((permission.requested == Lane::Left && !permission.left_allowed) ||
      (permission.requested == Lane::Right && !permission.right_allowed)) return invalid("REQUESTED_LANE_FORBIDDEN");
  if (observation.sections.size() < config.minimum_sections ||
      observation.sections.size() > config.maximum_sections) return invalid("SECTION_COUNT_INVALID");

  std::vector<LaneTarget> targets;
  targets.reserve(observation.sections.size());
  for (std::size_t index = 0; index < observation.sections.size(); ++index) {
    const auto& section = observation.sections[index];
    if (!std::isfinite(section.s) || !std::isfinite(section.reference_x) ||
        !std::isfinite(section.reference_y) || !std::isfinite(section.reference_yaw) ||
        std::abs(section.reference_yaw) > kPi + 1e-6) return invalid("SECTION_GEOMETRY_INVALID");
    if (section.observed_offsets_m.size() > config.maximum_points_per_section) return invalid("POINT_WORK_LIMIT");
    for (double d : section.observed_offsets_m) {
      if (!std::isfinite(d) || std::abs(d) > config.maximum_absolute_offset_m) return invalid("POINT_INVALID");
    }
    double step = 0;
    if (index > 0) {
      const auto& previous = observation.sections[index - 1];
      step = section.s - previous.s;
      if (!(step > 1e-4 && step <= config.maximum_section_gap_m)) return invalid("SECTION_GAP_OR_ORDER_INVALID");
      const double dx = section.reference_x - previous.reference_x;
      const double dy = section.reference_y - previous.reference_y;
      const double chord = std::hypot(dx, dy);
      const double chord_yaw = std::atan2(dy, dx);
      const double tangent_tolerance =
          config.maximum_reference_yaw_rate_per_m * step + 0.01;
      if (!std::isfinite(chord) || chord <= 1e-4 ||
          std::abs(chord - step) > config.maximum_reference_chord_error_m ||
          std::abs(yawDifference(section.reference_yaw, previous.reference_yaw)) >
              config.maximum_reference_yaw_rate_per_m * step ||
          std::abs(yawDifference(chord_yaw, previous.reference_yaw)) > tangent_tolerance ||
          std::abs(yawDifference(chord_yaw, section.reference_yaw)) > tangent_tolerance ||
          dx * std::cos(previous.reference_yaw) + dy * std::sin(previous.reference_yaw) <= 0.0) {
        return invalid("REFERENCE_DISCONTINUITY");
      }
    }
    const auto groups = clusters(section.observed_offsets_m, config);
    if (groups.size() > config.maximum_clusters_per_section) return invalid("CLUSTER_WORK_LIMIT");
    std::vector<std::pair<double, double>> pairs;
    for (std::size_t right = 0; right < groups.size(); ++right) {
      for (std::size_t left = right + 1; left < groups.size(); ++left) {
        // Inner faces, not cluster centroids: uncertainty does not expand the
        // apparent free road. The width prior validates, it never fabricates.
        const double rd = groups[right].high;
        const double ld = groups[left].low;
        const double width = ld - rd;
        if (std::abs(width - config.expected_road_width_m) <= config.road_width_tolerance_m) {
          pairs.emplace_back(ld, rd);
        }
      }
    }
    if (pairs.empty()) return invalid("BOTH_BOUNDARIES_NOT_OBSERVED");
    // Do not pick the widest/nearest/numerically best of several plausible roads.
    if (pairs.size() != 1) return invalid("AMBIGUOUS_BOUNDARIES");
    const double left = pairs[0].first;
    const double right = pairs[0].second;
    if (index > 0 &&
        (std::abs(left - targets.back().left_boundary_d) > config.maximum_boundary_slope * step + config.boundary_jitter_m ||
         std::abs(right - targets.back().right_boundary_d) > config.maximum_boundary_slope * step + config.boundary_jitter_m)) {
      return invalid("BOUNDARY_DISCONTINUITY");
    }
    const double width = left - right;
    if (width / 2.0 <= config.vehicle_width_m + 2.0 * config.lateral_margin_m) return invalid("LANE_TOO_NARROW");
    LaneTarget target;
    target.s = section.s;
    target.left_boundary_d = left;
    target.right_boundary_d = right;
    target.road_center_d = (left + right) / 2.0;
    target.left_lane_center_d = (3.0 * left + right) / 4.0;
    target.right_lane_center_d = (left + 3.0 * right) / 4.0;
    targets.push_back(target);
  }
  if (targets.back().s - targets.front().s < config.minimum_visible_length_m) return invalid("VISIBLE_LENGTH_TOO_SHORT");
  const double half_width = config.vehicle_width_m / 2.0 + config.lateral_margin_m;
  if (context.current_offset_m + half_width >= targets.front().left_boundary_d ||
      context.current_offset_m - half_width <= targets.front().right_boundary_d) return invalid("CURRENT_POSITION_OUTSIDE_CORRIDOR");

  Lane selected = permission.requested;
  if (selected == Lane::Unknown) {
    if (permission.left_allowed != permission.right_allowed) {
      selected = permission.left_allowed ? Lane::Left : Lane::Right;
    } else {
      // Keep the currently occupied lane whenever it is unambiguous. An
      // inconsistent caller label is rejected, not used to trigger a crossing.
      const double delta = context.current_offset_m - targets.front().road_center_d;
      const Lane geometric = delta > config.lane_tie_tolerance_m ? Lane::Left :
          delta < -config.lane_tie_tolerance_m ? Lane::Right : Lane::Unknown;
      if (context.current_lane != Lane::Unknown && geometric != Lane::Unknown &&
          context.current_lane != geometric) return invalid("CURRENT_LANE_CONTEXT_MISMATCH");
      selected = context.current_lane != Lane::Unknown ? context.current_lane : geometric;
      if (selected == Lane::Unknown) return invalid("LANE_CHOICE_AMBIGUOUS");
    }
  }
  for (std::size_t i = 0; i < targets.size(); ++i) {
    auto& target = targets[i];
    const auto& section = observation.sections[i];
    target.target_d = selected == Lane::Left ? target.left_lane_center_d : target.right_lane_center_d;
    target.target_x = section.reference_x - std::sin(section.reference_yaw) * target.target_d;
    target.target_y = section.reference_y + std::cos(section.reference_yaw) * target.target_d;
    if (!std::isfinite(target.target_x) || !std::isfinite(target.target_y)) return invalid("TARGET_NONFINITE");
  }
  Result result;
  result.valid = true;
  result.lane = selected;
  result.reason = "TARGET_ONLY_REQUIRES_TRAJECTORY_VALIDATION";
  result.targets = std::move(targets);
  return result;
}
}  // namespace frenet_lane_selection
