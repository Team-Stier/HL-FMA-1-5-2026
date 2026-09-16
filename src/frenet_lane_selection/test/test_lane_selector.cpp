#include "frenet_lane_selection/lane_selector.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>

namespace fls = frenet_lane_selection;
#define CHECK(condition) do { if (!(condition)) throw std::runtime_error( \
    std::string(__func__) + ":" + std::to_string(__LINE__) + " " #condition); } while (false)

fls::Observation road(double center = 0.0, bool curved = false) {
  fls::Observation observation;
  observation.input_contract_valid = true;
  for (int i = 0; i <= 8; ++i) {
    fls::CrossSection section;
    section.s = i * 0.5;
    section.reference_x = section.s;
    if (curved) {
      const double yaw = section.s / 12.0;
      section.reference_x = 12 * std::sin(yaw);
      section.reference_y = 12 * (1 - std::cos(yaw));
      section.reference_yaw = yaw;
    }
    section.observed_offsets_m = {center - 3.29, center - 3.28, center - 3.27,
                                  center + 3.27, center + 3.28, center + 3.29};
    observation.sections.push_back(section);
  }
  return observation;
}
fls::Permission permission(fls::Lane requested = fls::Lane::Left) {
  fls::Permission output;
  output.valid = true;
  output.left_allowed = true;
  output.right_allowed = true;
  output.requested = requested;
  return output;
}
void expectInvalid(const fls::Result& result, const std::string& reason) {
  CHECK(!result.valid);
  CHECK(result.targets.empty());
  CHECK(result.lane == fls::Lane::Unknown);
  CHECK(result.reason == reason);
}
void centersUseObservedRoadNotRddfOrigin() {
  const auto result = fls::selectLane(road(1.0), permission(), {});
  CHECK(result.valid);
  CHECK(std::abs(result.targets.front().road_center_d - 1.0) < 1e-9);
  CHECK(std::abs(result.targets.front().target_d - 2.635) < 1e-9);
  const auto right = fls::selectLane(road(1.0), permission(fls::Lane::Right), {});
  CHECK(right.valid);
  CHECK(std::abs(right.targets.front().target_d + .635) < 1e-9);
}
void curvesUseLocalNormals() {
  const auto input = road(0, true);
  const auto result = fls::selectLane(input, permission(), {});
  CHECK(result.valid);
  CHECK(std::abs(result.targets.back().target_x -
      (input.sections.back().reference_x - std::sin(4.0 / 12.0) * 1.635)) < 1e-9);
}
void permissionIsExternalAndCannotBeOverridden() {
  auto p = permission();
  p.valid = false;
  expectInvalid(fls::selectLane(road(), p, {}), "PERMISSION_INVALID_OR_STALE");
  p.valid = true; p.left_allowed = false;
  expectInvalid(fls::selectLane(road(), p, {}), "REQUESTED_LANE_FORBIDDEN");
  p.requested = fls::Lane::Unknown;
  CHECK(fls::selectLane(road(), p, {}).lane == fls::Lane::Right);
  p.right_allowed = false;
  expectInvalid(fls::selectLane(road(), p, {}), "NO_ALLOWED_LANE");
}
void bothAllowedKeepCurrentOrRefuseTie() {
  const auto p = permission(fls::Lane::Unknown);
  CHECK(fls::selectLane(road(), p, {1.0, fls::Lane::Unknown}).lane == fls::Lane::Left);
  CHECK(fls::selectLane(road(), p, {-1.0, fls::Lane::Right}).lane == fls::Lane::Right);
  CHECK(fls::selectLane(road(), p, {0.0, fls::Lane::Right}).lane == fls::Lane::Right);
  expectInvalid(fls::selectLane(road(), p, {}), "LANE_CHOICE_AMBIGUOUS");
  expectInvalid(fls::selectLane(road(), p, {1.0, fls::Lane::Right}), "CURRENT_LANE_CONTEXT_MISMATCH");
}
void oneSideOrOcclusionDoesNotInventWidth() {
  auto input = road();
  input.sections[4].observed_offsets_m.resize(3);
  expectInvalid(fls::selectLane(input, permission(), {}), "BOTH_BOUNDARIES_NOT_OBSERVED");
  input.sections[4].observed_offsets_m.clear();
  expectInvalid(fls::selectLane(input, permission(), {}), "BOTH_BOUNDARIES_NOT_OBSERVED");
}
void interiorPeopleRemainSurfacesNotRoadEdges() {
  auto input = road();
  for (auto& section : input.sections) {
    section.observed_offsets_m.insert(section.observed_offsets_m.end(), {.0, .01, .02});
  }
  const auto result = fls::selectLane(input, permission(), {});
  CHECK(result.valid);
  CHECK(std::abs(result.targets[0].target_d - 1.635) < 1e-9);
  // Interior points are NOT removed from the caller's collision observations.
  CHECK(input.sections.front().observed_offsets_m.size() == 9);
}
void multiplePlausibleRoadsRefused() {
  auto input = road();
  for (auto& section : input.sections) {
    section.observed_offsets_m.insert(section.observed_offsets_m.end(), {3.49, 3.50, 3.51});
  }
  expectInvalid(fls::selectLane(input, permission(), {}), "AMBIGUOUS_BOUNDARIES");
}
void rejectsWrongWidthAndBoundaryJumps() {
  auto input = road();
  for (auto& section : input.sections) {
    for (auto& d : section.observed_offsets_m) d *= .6;
  }
  expectInvalid(fls::selectLane(input, permission(), {}), "BOTH_BOUNDARIES_NOT_OBSERVED");
  input = road();
  for (auto& d : input.sections[4].observed_offsets_m) d += 1.0;
  expectInvalid(fls::selectLane(input, permission(), {}), "BOUNDARY_DISCONTINUITY");
}
void rejectsBadFrameTimingAndNumbers() {
  auto input = road();
  input.input_contract_valid = false;
  expectInvalid(fls::selectLane(input, permission(), {}), "INPUT_CONTRACT_INVALID");
  input = road(); input.age_sec = .51;
  expectInvalid(fls::selectLane(input, permission(), {}), "OBSERVATION_STALE");
  input.age_sec = -1;
  expectInvalid(fls::selectLane(input, permission(), {}), "OBSERVATION_STALE");
  input = road(); auto p = permission(); p.age_sec = .51;
  expectInvalid(fls::selectLane(input, p, {}), "PERMISSION_INVALID_OR_STALE");
  input.sections[0].observed_offsets_m[0] = std::numeric_limits<double>::quiet_NaN();
  expectInvalid(fls::selectLane(input, permission(), {}), "POINT_INVALID");
}
void rejectsBadCoverageOrAssociation() {
  auto input = road(); input.sections.resize(5);
  expectInvalid(fls::selectLane(input, permission(), {}), "SECTION_COUNT_INVALID");
  input = road(); input.sections[3].s += 1.0;
  expectInvalid(fls::selectLane(input, permission(), {}), "SECTION_GAP_OR_ORDER_INVALID");
  input = road(); input.sections[3].reference_x += 10;
  expectInvalid(fls::selectLane(input, permission(), {}), "REFERENCE_DISCONTINUITY");
  input = road(); input.sections[3].reference_yaw = 3;
  expectInvalid(fls::selectLane(input, permission(), {}), "REFERENCE_DISCONTINUITY");
}
void boundedWorkAndConfiguration() {
  auto input = road(); input.sections[0].observed_offsets_m.resize(257, 0.0);
  expectInvalid(fls::selectLane(input, permission(), {}), "POINT_WORK_LIMIT");
  fls::Config config; config.vehicle_width_m = 4.0;
  expectInvalid(fls::selectLane(road(), permission(), {}, config), "INVALID_CONFIG");
  config = {}; config.maximum_sections = 1000000;
  CHECK(!fls::validConfig(config));
  config = {}; config.cluster_gap_m = std::numeric_limits<double>::infinity();
  CHECK(!fls::validConfig(config));
}
void invalidNeverReusesTargets() {
  CHECK(fls::selectLane(road(), permission(), {}).valid);
  expectInvalid(fls::selectLane({}, permission(), {}), "INPUT_CONTRACT_INVALID");
  expectInvalid(fls::selectLane(road(), permission(), {3.0, fls::Lane::Left}), "CURRENT_POSITION_OUTSIDE_CORRIDOR");
}
void sCurveUsesArcStationAndNormals() {
  auto input = road();
  double station = 0;
  double previous_x = 0, previous_y = 0;
  for (std::size_t i = 0; i < input.sections.size(); ++i) {
    auto& section = input.sections[i];
    const double x = i * .5;
    const double y = .2 * std::sin(x / .9);
    if (i > 0) station += std::hypot(x - previous_x, y - previous_y);
    section.s = station;
    section.reference_x = x;
    section.reference_y = y;
    section.reference_yaw = std::atan((.2 / .9) * std::cos(x / .9));
    previous_x = x; previous_y = y;
  }
  const auto result = fls::selectLane(input, permission(fls::Lane::Right), {});
  CHECK(result.valid);
  for (std::size_t i = 0; i < result.targets.size(); ++i) {
    const auto& target = result.targets[i]; const auto& section = input.sections[i];
    CHECK(std::abs(target.target_d + 1.635) < 1e-9);
    CHECK(std::abs(std::hypot(target.target_x - section.reference_x,
                             target.target_y - section.reference_y) - 1.635) < 1e-9);
  }
}
void deterministicGeometryProperties() {
  std::mt19937 generator(20260914);
  std::uniform_real_distribution<double> center(-.8, .8);
  std::uniform_real_distribution<double> rotation(-3., 3.);
  std::uniform_real_distribution<double> noise(-.005, .005);
  for (int sample = 0; sample < 500; ++sample) {
    const double offset = center(generator), yaw = rotation(generator);
    auto input = road(offset);
    for (auto& section : input.sections) {
      section.reference_x = 10 + section.s * std::cos(yaw);
      section.reference_y = -20 + section.s * std::sin(yaw);
      section.reference_yaw = yaw;
      for (auto& d : section.observed_offsets_m) d += noise(generator);
    }
    const auto lane = sample % 2 ? fls::Lane::Left : fls::Lane::Right;
    const auto result = fls::selectLane(input, permission(lane), {offset, fls::Lane::Unknown});
    CHECK(result.valid); CHECK(result.lane == lane);
    for (const auto& target : result.targets) {
      CHECK(target.target_d > target.right_boundary_d + .6375);
      CHECK(target.target_d < target.left_boundary_d - .6375);
      CHECK(std::abs(target.road_center_d - offset) < .006);
    }
  }
}
int main() {
  const std::pair<const char*, void(*)()> tests[] = {
      {"centers", centersUseObservedRoadNotRddfOrigin}, {"curve", curvesUseLocalNormals},
      {"permissions", permissionIsExternalAndCannotBeOverridden}, {"keep_or_tie", bothAllowedKeepCurrentOrRefuseTie},
      {"occlusion", oneSideOrOcclusionDoesNotInventWidth}, {"clutter", interiorPeopleRemainSurfacesNotRoadEdges},
      {"ambiguity", multiplePlausibleRoadsRefused}, {"width_jump", rejectsWrongWidthAndBoundaryJumps},
      {"input", rejectsBadFrameTimingAndNumbers}, {"coverage", rejectsBadCoverageOrAssociation},
      {"work_config", boundedWorkAndConfiguration}, {"no_stale", invalidNeverReusesTargets},
      {"s_curve", sCurveUsesArcStationAndNormals}, {"properties_500", deterministicGeometryProperties}};
  for (const auto& test : tests) {
    try { test.second(); std::cout << "PASS " << test.first << '\n'; }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
  }
  std::cout << "14 scenario groups passed (including 500 geometry cases); synthetic inputs only\n";
}
