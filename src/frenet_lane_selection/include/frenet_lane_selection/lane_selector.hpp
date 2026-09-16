#pragma once

#include <cstddef>
#include <string>
#include <vector>

// ROS-free geometry/decision core. A valid result is a TARGET, not a drivable
// trajectory, a semantic curb label, or permission to actuate a vehicle.
namespace frenet_lane_selection {

enum class Lane { Unknown = 0, Left = 1, Right = 2 };

struct Config {
  double expected_road_width_m{6.54};
  double road_width_tolerance_m{0.30};
  double cluster_gap_m{0.08};
  double maximum_cluster_span_m{0.15};
  std::size_t minimum_cluster_points{3};
  std::size_t minimum_sections{6};
  std::size_t maximum_sections{64};
  std::size_t maximum_points_per_section{256};
  std::size_t maximum_clusters_per_section{24};
  double minimum_visible_length_m{3.0};
  double maximum_section_gap_m{1.0};
  double maximum_boundary_slope{0.35};
  double boundary_jitter_m{0.05};
  double maximum_reference_yaw_rate_per_m{0.60};
  double maximum_reference_chord_error_m{0.08};
  double maximum_absolute_offset_m{12.0};
  double vehicle_width_m{0.775};
  double lateral_margin_m{0.25};
  double maximum_observation_age_sec{0.50};
  double maximum_permission_age_sec{0.50};
  double lane_tie_tolerance_m{0.10};
};

// The integrator projects measurement-time LiDAR points ONLY onto the active
// route segment. Each section is a narrow longitudinal bin of those points.
// Offsets are signed: left of the reference tangent is positive. Provide all
// retained surfaces, not just points pre-labelled as curbs. TF, deskew, the
// longitudinal association window and point identity are external contracts.
struct CrossSection {
  double s{0.0};
  double reference_x{0.0};
  double reference_y{0.0};
  double reference_yaw{0.0};
  std::vector<double> observed_offsets_m;
};

struct Observation {
  // Explicit opt-in after frame/timing/route association checks. Never infer
  // this from "there are points". Ages must advance even when a ROS clock stops.
  bool input_contract_valid{false};
  double age_sec{0.0};
  std::vector<CrossSection> sections;
};

struct Permission {
  // Supplied by mission/sign recognition or a deliberate test fixture. A curb
  // says nothing about whether an arrow or X allows a particular lane.
  bool valid{false};
  double age_sec{0.0};
  bool left_allowed{false};
  bool right_allowed{false};
  Lane requested{Lane::Unknown};
};

struct SelectionContext {
  // Lateral position at the FIRST section, not at an unrelated ego station.
  // The integrator must establish this association; no global nearest match.
  double current_offset_m{0.0};
  Lane current_lane{Lane::Unknown};
};

struct LaneTarget {
  double s{0.0};
  double left_boundary_d{0.0};
  double right_boundary_d{0.0};
  double road_center_d{0.0};
  double left_lane_center_d{0.0};
  double right_lane_center_d{0.0};
  double target_d{0.0};
  double target_x{0.0};
  double target_y{0.0};
};

struct Result {
  bool valid{false};
  Lane lane{Lane::Unknown};
  std::string reason;
  std::vector<LaneTarget> targets;
};

bool validConfig(const Config& config);
// Stateless: failed calls return NO targets and never reuse a previous result.
// Both boundaries must be observed in EVERY section. Ambiguity, occlusion and
// bad width fail closed rather than completing a road from the 6.54 m prior.
Result selectLane(const Observation& observation, const Permission& permission,
                  const SelectionContext& context, const Config& config = Config{});

}  // namespace frenet_lane_selection
