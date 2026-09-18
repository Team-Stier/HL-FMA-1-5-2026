#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <planning_interfaces/MissionState.h>
#include <planning_interfaces/PathStatus.h>
#include <planning_interfaces/PlannedPath.h>
#include <planning_interfaces/RouteMap.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "path_planner/frenet_planner.hpp"
#include "path_planner/obstacle_projection.hpp"
#include "path_planner/pose_jump_guard.hpp"
#include "path_planner/reference_path.hpp"

namespace {

bool finite(double value) { return std::isfinite(value); }

bool validQuaternion(const geometry_msgs::Quaternion& value) {
  const double norm = value.x * value.x + value.y * value.y +
                      value.z * value.z + value.w * value.w;
  return finite(norm) && std::abs(norm - 1.0) <= 0.002;
}

class PathPlannerNode {
 public:
  PathPlannerNode() : private_node_("~"), jump_guard_(loadJumpConfig()) {
    loadConfiguration();

    path_publisher_ = node_.advertise<planning_interfaces::PlannedPath>(
        "/path/local", 1);
    visualization_publisher_ = node_.advertise<nav_msgs::Path>(
        "/path/local_visualization", 1);
    status_publisher_ = node_.advertise<planning_interfaces::PathStatus>(
        "/path_planner/status", 1);
    route_subscriber_ = node_.subscribe(
        "/route/map", 1, &PathPlannerNode::routeCallback, this);
    mission_subscriber_ = node_.subscribe(
        "/mission/state", 1, &PathPlannerNode::missionCallback, this);
    odometry_subscriber_ = node_.subscribe(
        "/molit/localization/odometry", 1,
        &PathPlannerNode::odometryCallback, this);
    obstacle_subscriber_ = node_.subscribe(
        obstacle_topic_, 1, &PathPlannerNode::obstacleCallback, this);
    timer_ = node_.createTimer(ros::Duration(1.0 / planning_rate_hz_),
                               &PathPlannerNode::timerCallback, this);

    if (configuration_ready_) {
      ROS_INFO("path_planner ready: obstacles=%s output=/path/local",
               obstacle_topic_.c_str());
    } else {
      ROS_WARN("path_planner is connected but inhibited: %s",
               configuration_reason_.c_str());
    }
  }

 private:
  path_planner::PoseJumpGuardConfig loadJumpConfig() {
    path_planner::PoseJumpGuardConfig value;
    private_node_.param("pose_jump/translation_tolerance_m",
                        value.translation_tolerance_m,
                        value.translation_tolerance_m);
    private_node_.param("pose_jump/yaw_tolerance_rad", value.yaw_tolerance_rad,
                        value.yaw_tolerance_rad);
    private_node_.param("pose_jump/maximum_speed_mps", value.maximum_speed_mps,
                        value.maximum_speed_mps);
    private_node_.param("pose_jump/maximum_yaw_rate_rad_per_sec",
                        value.maximum_yaw_rate_rad_per_sec,
                        value.maximum_yaw_rate_rad_per_sec);
    private_node_.param("pose_jump/maximum_pose_interval_sec",
                        value.maximum_pose_interval_sec,
                        value.maximum_pose_interval_sec);
    return value;
  }

  void loadConfiguration() {
    private_node_.param("frame_id", frame_id_, std::string("map"));
    private_node_.param("planning_rate_hz", planning_rate_hz_, 5.0);
    private_node_.param("input_timeout_s", input_timeout_s_, 0.5);
    private_node_.param("future_tolerance_s", future_tolerance_s_, 0.05);
    private_node_.param("planning_deadline_ms", planning_deadline_ms_, 80.0);
    private_node_.param("calibration_required", calibration_required_, true);
    private_node_.param("road_left_bound_m", road_left_bound_m_, 0.0);
    private_node_.param("road_right_bound_m", road_right_bound_m_, 0.0);
    private_node_.param("projection/back_window_m", projection_back_window_m_,
                        3.0);
    private_node_.param("projection/forward_window_m",
                        projection_forward_window_m_, 15.0);
    private_node_.param("projection/heading_weight", projection_heading_weight_,
                        2.0);
    private_node_.param("projection/maximum_pose_distance_m",
                        maximum_pose_distance_m_, 2.0);
    private_node_.param("projection/obstacle_maximum_distance_m",
                        obstacle_maximum_distance_m_, 5.0);
    private_node_.param("projection/obstacle_station_tolerance_m",
                        obstacle_station_tolerance_m_, 4.0);
    private_node_.param("obstacles/topic", obstacle_topic_,
                        std::string("/dbscan_clusters"));
    private_node_.param("obstacles/namespace", obstacle_namespace_,
                        std::string("dbscan_clusters"));
    private_node_.param("obstacles/minimum_extent_m", minimum_obstacle_extent_m_,
                        0.10);
    private_node_.param("obstacles/maximum_extent_m", maximum_obstacle_extent_m_,
                        5.0);
    private_node_.param("obstacles/maximum_count", maximum_obstacle_count_, 100);

    private_node_.param("vehicle/length_m", planner_config_.vehicle.length_m,
                        0.0);
    private_node_.param("vehicle/width_m", planner_config_.vehicle.width_m,
                        0.0);
    private_node_.param("vehicle/wheelbase_m", planner_config_.vehicle.wheelbase_m,
                        0.0);
    private_node_.param("vehicle/rear_axle_to_center_m",
                        planner_config_.vehicle.rear_axle_to_center_m, 0.0);
    private_node_.param("planner/horizon_m", planner_config_.horizon_m,
                        planner_config_.horizon_m);
    private_node_.param("planner/sample_interval_m",
                        planner_config_.sample_interval_m,
                        planner_config_.sample_interval_m);
    private_node_.param("planner/lateral_sample_interval_m",
                        planner_config_.lateral_sample_interval_m,
                        planner_config_.lateral_sample_interval_m);
    private_node_.param("planner/maximum_lateral_offset_m",
                        planner_config_.maximum_lateral_offset_m,
                        planner_config_.maximum_lateral_offset_m);
    private_node_.param("planner/collision_margin_m",
                        planner_config_.collision_margin_m,
                        planner_config_.collision_margin_m);
    private_node_.param("planner/boundary_margin_m",
                        planner_config_.boundary_margin_m,
                        planner_config_.boundary_margin_m);
    private_node_.param("planner/maximum_curvature_per_m",
                        planner_config_.maximum_curvature_per_m,
                        planner_config_.maximum_curvature_per_m);
    private_node_.param("planner/maximum_curvature_rate_per_m2",
                        planner_config_.maximum_curvature_rate_per_m2,
                        planner_config_.maximum_curvature_rate_per_m2);
    private_node_.param("planner/maximum_lateral_slope",
                        planner_config_.maximum_lateral_slope,
                        planner_config_.maximum_lateral_slope);
    private_node_.param("planner/minimum_longitudinal_scale",
                        planner_config_.minimum_longitudinal_scale,
                        planner_config_.minimum_longitudinal_scale);
    private_node_.param("planner/boundary_perimeter_sample_interval_m",
                        planner_config_.boundary_perimeter_sample_interval_m,
                        planner_config_.boundary_perimeter_sample_interval_m);

    configuration_ready_ = !calibration_required_;
    configuration_reason_ = "MEASURED_PLANNER_CALIBRATION_REQUIRED";
    std::string planner_error;
    const bool scalar_parameters_valid =
        finite(planning_rate_hz_) && planning_rate_hz_ > 0.0 &&
        planning_rate_hz_ <= 50.0 && finite(input_timeout_s_) &&
        input_timeout_s_ > 0.0 && finite(future_tolerance_s_) &&
        future_tolerance_s_ >= 0.0 && finite(planning_deadline_ms_) &&
        planning_deadline_ms_ > 0.0 && finite(road_left_bound_m_) &&
        road_left_bound_m_ > 0.0 && finite(road_right_bound_m_) &&
        road_right_bound_m_ > 0.0 && finite(projection_back_window_m_) &&
        projection_back_window_m_ > 0.0 &&
        finite(projection_forward_window_m_) &&
        projection_forward_window_m_ > 0.0 &&
        finite(projection_heading_weight_) && projection_heading_weight_ >= 0.0 &&
        finite(maximum_pose_distance_m_) && maximum_pose_distance_m_ > 0.0 &&
        finite(minimum_obstacle_extent_m_) && minimum_obstacle_extent_m_ > 0.0 &&
        finite(maximum_obstacle_extent_m_) &&
        maximum_obstacle_extent_m_ >= minimum_obstacle_extent_m_ &&
        maximum_obstacle_count_ > 0;
    if (!calibration_required_ &&
        (!scalar_parameters_valid ||
         !path_planner::isValid(planner_config_, &planner_error))) {
      configuration_ready_ = false;
      configuration_reason_ = "INVALID_PLANNER_CONFIGURATION";
      if (!planner_error.empty()) {
        configuration_reason_ += ":" + planner_error;
      }
    }
    if (configuration_ready_) {
      planner_.reset(new path_planner::FrenetPlanner(planner_config_));
    }
  }

  void resetPlanner() {
    if (planner_) {
      planner_->reset();
    }
  }

  void routeCallback(const planning_interfaces::RouteMap::ConstPtr& message) {
    if (!configuration_ready_) {
      return;
    }
    if (message->header.frame_id != frame_id_) {
      routes_.clear();
      route_error_ = "ROUTE_MAP_FRAME_MISMATCH";
      resetPlanner();
      return;
    }
    std::unordered_map<std::string, path_planner::ReferencePath> routes;
    for (const planning_interfaces::Route& source : message->routes) {
      if (source.name.empty() || routes.count(source.name) != 0U ||
          source.path.header.frame_id != frame_id_) {
        routes_.clear();
        route_error_ = "ROUTE_MAP_METADATA_INVALID";
        resetPlanner();
        return;
      }
      std::vector<path_planner::Waypoint> waypoints;
      waypoints.reserve(source.path.poses.size());
      for (const geometry_msgs::PoseStamped& pose : source.path.poses) {
        if (pose.header.frame_id != frame_id_ ||
            !finite(pose.pose.position.x) || !finite(pose.pose.position.y)) {
          routes_.clear();
          route_error_ = "ROUTE_MAP_POINT_INVALID";
          resetPlanner();
          return;
        }
        waypoints.push_back({pose.pose.position.x, pose.pose.position.y,
                             road_left_bound_m_, road_right_bound_m_, 0.0,
                             "STATIC_AVOIDANCE"});
      }
      path_planner::ReferencePath reference;
      std::string error;
      if (!reference.initialize(waypoints, &error)) {
        routes_.clear();
        route_error_ = "ROUTE_MAP_GEOMETRY_INVALID:" + source.name + ":" + error;
        resetPlanner();
        return;
      }
      routes.emplace(source.name, std::move(reference));
    }
    routes_ = std::move(routes);
    route_error_.clear();
    resetPlanner();
    jump_guard_.reset();
  }

  void missionCallback(
      const planning_interfaces::MissionState::ConstPtr& message) {
    const bool changed = !mission_received_ ||
                         mission_.decision_id != message->decision_id ||
                         mission_.route_name != message->route_name ||
                         mission_.path_mode != message->path_mode ||
                         mission_.direction != message->direction;
    mission_ = *message;
    mission_receipt_ = ros::Time::now();
    mission_received_ = true;
    if (changed) {
      resetPlanner();
    }
  }

  void odometryCallback(const nav_msgs::Odometry::ConstPtr& message) {
    const geometry_msgs::Pose& source = message->pose.pose;
    if (message->header.frame_id != frame_id_ || message->header.stamp.isZero() ||
        !finite(source.position.x) || !finite(source.position.y) ||
        !validQuaternion(source.orientation)) {
      odometry_valid_ = false;
      return;
    }
    path_planner::Pose2d pose{source.position.x, source.position.y,
                              tf2::getYaw(source.orientation)};
    std::string reason;
    if (jump_guard_.updatePose(pose, message->header.stamp.toSec(), &reason)) {
      odometry_valid_ = false;
      obstacles_valid_ = false;
      resetPlanner();
      pose_error_ = reason;
      return;
    }
    pose_ = pose;
    odometry_stamp_ = message->header.stamp;
    odometry_receipt_ = ros::Time::now();
    odometry_valid_ = true;
    pose_error_.clear();
  }

  void obstacleCallback(
      const visualization_msgs::MarkerArray::ConstPtr& message) {
    ros::Time stamp;
    std::string frame;
    for (const visualization_msgs::Marker& marker : message->markers) {
      if (!marker.header.stamp.isZero() && !marker.header.frame_id.empty()) {
        if (stamp.isZero()) {
          stamp = marker.header.stamp;
          frame = marker.header.frame_id;
        } else if (marker.header.stamp != stamp ||
                   marker.header.frame_id != frame) {
          obstacles_valid_ = false;
          obstacle_error_ = "OBSTACLE_ARRAY_HEADER_MISMATCH";
          return;
        }
      }
    }
    if (stamp.isZero() || frame != frame_id_) {
      obstacles_valid_ = false;
      obstacle_error_ = "OBSTACLE_HEARTBEAT_INVALID";
      return;
    }

    std::vector<path_planner::Obstacle2d> obstacles;
    for (const visualization_msgs::Marker& marker : message->markers) {
      if (marker.action == visualization_msgs::Marker::DELETEALL ||
          marker.ns != obstacle_namespace_) {
        continue;
      }
      if (marker.action != visualization_msgs::Marker::ADD ||
          marker.type != visualization_msgs::Marker::POINTS ||
          marker.header.stamp != stamp || marker.header.frame_id != frame ||
          !validQuaternion(marker.pose.orientation)) {
        obstacles_valid_ = false;
        obstacle_error_ = "OBSTACLE_MARKER_INVALID";
        return;
      }
      if (marker.points.empty()) {
        continue;
      }
      const double yaw = tf2::getYaw(marker.pose.orientation);
      const double cosine = std::cos(yaw);
      const double sine = std::sin(yaw);
      double minimum_x = std::numeric_limits<double>::infinity();
      double maximum_x = -std::numeric_limits<double>::infinity();
      double minimum_y = std::numeric_limits<double>::infinity();
      double maximum_y = -std::numeric_limits<double>::infinity();
      for (const geometry_msgs::Point& point : marker.points) {
        if (!finite(point.x) || !finite(point.y)) {
          obstacles_valid_ = false;
          obstacle_error_ = "OBSTACLE_POINT_NONFINITE";
          return;
        }
        const double x = marker.pose.position.x +
                         cosine * point.x - sine * point.y;
        const double y = marker.pose.position.y +
                         sine * point.x + cosine * point.y;
        minimum_x = std::min(minimum_x, x);
        maximum_x = std::max(maximum_x, x);
        minimum_y = std::min(minimum_y, y);
        maximum_y = std::max(maximum_y, y);
      }
      double extent_x = maximum_x - minimum_x;
      double extent_y = maximum_y - minimum_y;
      if (extent_x > maximum_obstacle_extent_m_ ||
          extent_y > maximum_obstacle_extent_m_) {
        obstacles_valid_ = false;
        obstacle_error_ = "OBSTACLE_EXTENT_INVALID";
        return;
      }
      const double center_x = (minimum_x + maximum_x) * 0.5;
      const double center_y = (minimum_y + maximum_y) * 0.5;
      extent_x = std::max(extent_x, minimum_obstacle_extent_m_);
      extent_y = std::max(extent_y, minimum_obstacle_extent_m_);
      const double half_x = extent_x * 0.5;
      const double half_y = extent_y * 0.5;
      obstacles.push_back({
          {center_x, center_y},
          {{{center_x - half_x, center_y - half_y},
            {center_x + half_x, center_y - half_y},
            {center_x + half_x, center_y + half_y},
            {center_x - half_x, center_y + half_y}}}});
      if (static_cast<int>(obstacles.size()) > maximum_obstacle_count_) {
        obstacles_valid_ = false;
        obstacle_error_ = "OBSTACLE_COUNT_EXCEEDED";
        return;
      }
    }
    obstacles_ = std::move(obstacles);
    obstacle_stamp_ = stamp;
    obstacle_receipt_ = ros::Time::now();
    obstacles_valid_ = true;
    obstacle_error_.clear();
  }

  bool fresh(const ros::Time& source_stamp, const ros::Time& receipt_stamp,
             const ros::Time& now) const {
    if (source_stamp.isZero() || receipt_stamp.isZero()) {
      return false;
    }
    const double source_age = (now - source_stamp).toSec();
    const double receipt_age = (now - receipt_stamp).toSec();
    return finite(source_age) && finite(receipt_age) &&
           source_age >= -future_tolerance_s_ &&
           source_age <= input_timeout_s_ && receipt_age >= 0.0 &&
           receipt_age <= input_timeout_s_;
  }

  void publishStatus(const ros::Time& now, bool ready,
                     const std::string& reason) {
    planning_interfaces::PathStatus status;
    status.header.stamp = now;
    status.header.frame_id = frame_id_;
    status.source = "LOCAL";
    status.ready = ready;
    status.reason = reason;
    if (mission_received_) {
      status.decision_id = mission_.decision_id;
      status.route_name = mission_.route_name;
      status.direction = mission_.direction;
    }
    status_publisher_.publish(status);
    if (!ready) {
      nav_msgs::Path empty;
      empty.header = status.header;
      visualization_publisher_.publish(empty);
    }
  }

  void timerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    if (!configuration_ready_) {
      publishStatus(now, false, configuration_reason_);
      return;
    }
    if (!mission_received_ ||
        !fresh(mission_.header.stamp, mission_receipt_, now)) {
      publishStatus(now, false, "MISSION_STATE_STALE");
      return;
    }
    if (mission_.path_mode != "LOCAL") {
      publishStatus(now, false, "IDLE_NOT_LOCAL_MODE");
      return;
    }
    if (!mission_.valid || mission_.direction != 1) {
      publishStatus(now, false, "LOCAL_REQUEST_INVALID_OR_NOT_FORWARD");
      return;
    }
    const auto route = routes_.find(mission_.route_name);
    if (route == routes_.end()) {
      publishStatus(now, false,
                    route_error_.empty() ? "ACTIVE_ROUTE_UNAVAILABLE"
                                         : route_error_);
      return;
    }
    if (!odometry_valid_ ||
        !fresh(odometry_stamp_, odometry_receipt_, now)) {
      publishStatus(now, false,
                    pose_error_.empty() ? "ODOMETRY_STALE" : pose_error_);
      return;
    }
    if (!obstacles_valid_ ||
        !fresh(obstacle_stamp_, obstacle_receipt_, now)) {
      publishStatus(now, false, obstacle_error_.empty()
                                    ? "OBSTACLE_OBSERVATION_STALE"
                                    : obstacle_error_);
      return;
    }

    const path_planner::ReferencePath& reference = route->second;
    const double expected_s = std::max(
        0.0, std::min(reference.length(), mission_.distance_m));
    const double minimum_s = std::max(0.0, expected_s - projection_back_window_m_);
    const double maximum_s = std::min(
        reference.length(), expected_s + projection_forward_window_m_);
    const path_planner::Projection projection = reference.projectPose(
        pose_, minimum_s, maximum_s, projection_heading_weight_);
    if (!projection.valid || projection.distance_m > maximum_pose_distance_m_) {
      resetPlanner();
      publishStatus(now, false, "POSE_NOT_ON_ACTIVE_ROUTE");
      return;
    }

    path_planner::PlannerInput input;
    input.pose = pose_;
    input.projection = projection;
    input.obstacles = obstacles_;
    input.frenet_obstacles = path_planner::projectObstaclesToFrenet(
        reference, obstacles_, minimum_s, maximum_s,
        obstacle_maximum_distance_m_, projection.s,
        obstacle_station_tolerance_m_);

    const auto started = std::chrono::steady_clock::now();
    const path_planner::PlannerResult result = planner_->plan(reference, input);
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - started)
            .count();
    if (elapsed_ms > planning_deadline_ms_) {
      ROS_WARN_THROTTLE(
          2.0,
          "Frenet planning took %.2f ms (warning threshold %.2f ms)",
          elapsed_ms, planning_deadline_ms_);
    }
    if (!result.valid || result.path.size() < 3U) {
      resetPlanner();
      publishStatus(now, false, "NO_VALID_PATH:" + result.reason);
      return;
    }

    planning_interfaces::PlannedPath output;
    output.header.stamp = now;
    output.header.frame_id = frame_id_;
    output.decision_id = mission_.decision_id;
    output.route_name = mission_.route_name;
    output.direction = mission_.direction;
    output.path.header = output.header;
    output.path.poses.reserve(result.path.size());
    for (const path_planner::PathPoint& point : result.path) {
      if (!finite(point.x) || !finite(point.y) || !finite(point.yaw)) {
        resetPlanner();
        publishStatus(now, false, "PLANNER_OUTPUT_NONFINITE");
        return;
      }
      geometry_msgs::PoseStamped pose;
      pose.header = output.header;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.orientation.z = std::sin(point.yaw * 0.5);
      pose.pose.orientation.w = std::cos(point.yaw * 0.5);
      output.path.poses.push_back(pose);
    }
    path_publisher_.publish(output);
    visualization_publisher_.publish(output.path);
    publishStatus(now, true, result.reason);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Publisher path_publisher_;
  ros::Publisher visualization_publisher_;
  ros::Publisher status_publisher_;
  ros::Subscriber route_subscriber_;
  ros::Subscriber mission_subscriber_;
  ros::Subscriber odometry_subscriber_;
  ros::Subscriber obstacle_subscriber_;
  ros::Timer timer_;

  std::string frame_id_;
  std::string obstacle_topic_;
  std::string obstacle_namespace_;
  double planning_rate_hz_{5.0};
  double input_timeout_s_{0.5};
  double future_tolerance_s_{0.05};
  double planning_deadline_ms_{80.0};
  bool calibration_required_{true};
  bool configuration_ready_{false};
  std::string configuration_reason_;
  double road_left_bound_m_{0.0};
  double road_right_bound_m_{0.0};
  double projection_back_window_m_{3.0};
  double projection_forward_window_m_{15.0};
  double projection_heading_weight_{2.0};
  double maximum_pose_distance_m_{2.0};
  double obstacle_maximum_distance_m_{5.0};
  double obstacle_station_tolerance_m_{4.0};
  double minimum_obstacle_extent_m_{0.10};
  double maximum_obstacle_extent_m_{5.0};
  int maximum_obstacle_count_{100};

  path_planner::FrenetPlannerConfig planner_config_;
  std::unique_ptr<path_planner::FrenetPlanner> planner_;
  path_planner::PoseJumpGuard jump_guard_;
  std::unordered_map<std::string, path_planner::ReferencePath> routes_;
  std::string route_error_;

  planning_interfaces::MissionState mission_;
  bool mission_received_{false};
  ros::Time mission_receipt_;
  path_planner::Pose2d pose_;
  bool odometry_valid_{false};
  ros::Time odometry_stamp_;
  ros::Time odometry_receipt_;
  std::string pose_error_;
  std::vector<path_planner::Obstacle2d> obstacles_;
  bool obstacles_valid_{false};
  ros::Time obstacle_stamp_;
  ros::Time obstacle_receipt_;
  std::string obstacle_error_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_planner");
  PathPlannerNode node;
  ros::spin();
  return 0;
}
