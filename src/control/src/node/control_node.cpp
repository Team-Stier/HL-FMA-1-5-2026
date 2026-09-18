#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <erp42_msgs/DriveCmd.h>
#include <erp42_msgs/SerialFeedBack.h>
#include <planning_interfaces/MissionState.h>

#include "control/lateral/pure_pursuit.hpp"
#include "control/lateral/stanley_controller.hpp"
#include "control/longitudinal/fixed_speed_controller.hpp"
#include "control/vehicle/t870_command_mapper.hpp"

namespace stier_control {
namespace {

constexpr double kPi = 3.14159265358979323846;

double degreesToRadians(double degrees) {
  return degrees * kPi / 180.0;
}

double clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(upper, value));
}

bool positiveFinite(double value) {
  return std::isfinite(value) && value > 0.0;
}

template <typename T>
T requiredParam(const ros::NodeHandle& node, const std::string& name) {
  T value;
  if (!node.getParam(name, value)) {
    throw std::runtime_error("missing required private parameter: " + name);
  }
  return value;
}

struct NodeConfig {
  std::string path_topic;
  std::string mission_topic;
  std::string odometry_topic;
  std::string feedback_topic;
  std::string emergency_stop_topic;
  std::string command_topic;
  std::string state_topic;
  std::string lookahead_point_topic;
  std::string stanley_projection_point_topic;
  std::string cross_track_error_topic;
  std::string heading_error_topic;
  std::string steering_angle_topic;
  std::string expected_frame_id;
  std::string vehicle_frame_id;
  std::string controller_mode;
  bool calibration_required{false};
  bool require_ros_mode{true};
  double control_rate_hz{0.0};
  double path_timeout_sec{0.0};
  double mission_timeout_sec{0.0};
  double odometry_timeout_sec{0.0};
  double feedback_timeout_sec{0.0};
  double maximum_control_dt_sec{0.0};
  double wheelbase_m{0.0};
  double rear_axle_to_pose_reference_m{0.0};
  double curvature_preview_distance_m{0.0};
  double maximum_steering_rate_rad_per_sec{0.0};
  int parking_target_speed_kph{0};
  int parallel_parking_target_speed_kph{0};
  FixedSpeedConfig fixed_speed;
  PurePursuitConfig pure_pursuit;
  StanleyConfig stanley;
  T870SteeringCommandConfig command_mapper;
};

NodeConfig loadConfig(const ros::NodeHandle& node) {
  NodeConfig config;
  config.path_topic = requiredParam<std::string>(node, "path_topic");
  config.mission_topic = requiredParam<std::string>(node, "mission_topic");
  config.odometry_topic = requiredParam<std::string>(node, "odometry_topic");
  config.feedback_topic =
      requiredParam<std::string>(node, "feedback_topic");
  config.emergency_stop_topic =
      requiredParam<std::string>(node, "emergency_stop_topic");
  config.command_topic = requiredParam<std::string>(node, "command_topic");
  config.state_topic = requiredParam<std::string>(node, "state_topic");
  config.lookahead_point_topic =
      requiredParam<std::string>(node, "lookahead_point_topic");
  config.stanley_projection_point_topic =
      requiredParam<std::string>(node, "stanley_projection_point_topic");
  config.cross_track_error_topic =
      requiredParam<std::string>(node, "cross_track_error_topic");
  config.heading_error_topic =
      requiredParam<std::string>(node, "heading_error_topic");
  config.steering_angle_topic =
      requiredParam<std::string>(node, "steering_angle_topic");
  config.expected_frame_id =
      requiredParam<std::string>(node, "expected_frame_id");
  config.vehicle_frame_id =
      requiredParam<std::string>(node, "vehicle_frame_id");
  config.controller_mode =
      requiredParam<std::string>(node, "lateral_controller");
  config.calibration_required =
      requiredParam<bool>(node, "calibration_required");
  config.require_ros_mode = requiredParam<bool>(node, "require_ros_mode");
  config.control_rate_hz = requiredParam<double>(node, "control_rate_hz");
  config.path_timeout_sec = requiredParam<double>(node, "path_timeout_sec");
  config.mission_timeout_sec =
      requiredParam<double>(node, "mission_timeout_sec");
  config.odometry_timeout_sec =
      requiredParam<double>(node, "odometry_timeout_sec");
  config.feedback_timeout_sec =
      requiredParam<double>(node, "feedback_timeout_sec");
  config.maximum_control_dt_sec =
      requiredParam<double>(node, "maximum_control_dt_sec");
  config.wheelbase_m = requiredParam<double>(node, "wheelbase_m");
  config.rear_axle_to_pose_reference_m =
      requiredParam<double>(node, "rear_axle_to_pose_reference_m");
  config.curvature_preview_distance_m =
      requiredParam<double>(node, "curvature_preview_distance_m");
  config.maximum_steering_rate_rad_per_sec = degreesToRadians(
      requiredParam<double>(node, "maximum_steering_rate_deg_per_sec"));

  config.fixed_speed.target_speed_kph =
      requiredParam<int>(node, "target_speed_kph");
  config.parking_target_speed_kph =
      requiredParam<int>(node, "parking_target_speed_kph");
  config.parallel_parking_target_speed_kph =
      requiredParam<int>(node, "parallel_parking_target_speed_kph");
  config.fixed_speed.maximum_speed_kph =
      requiredParam<int>(node, "maximum_speed_kph");
  if (!isValidFixedSpeedConfig(config.fixed_speed)) {
    throw std::runtime_error("invalid T870 target/maximum speed parameters");
  }
  if (config.parking_target_speed_kph <= 0 ||
      config.parking_target_speed_kph > config.fixed_speed.maximum_speed_kph) {
    throw std::runtime_error("invalid parking target speed parameter");
  }
  if (config.parallel_parking_target_speed_kph <= 0 ||
      config.parallel_parking_target_speed_kph >
          config.fixed_speed.maximum_speed_kph) {
    throw std::runtime_error("invalid parallel parking target speed parameter");
  }

  config.pure_pursuit.wheelbase_m = config.wheelbase_m;
  config.pure_pursuit.lookahead_base_m =
      requiredParam<double>(node, "lookahead_base_m");
  config.pure_pursuit.lookahead_speed_gain_sec =
      requiredParam<double>(node, "lookahead_speed_gain_sec");
  config.pure_pursuit.lookahead_curvature_gain_m =
      requiredParam<double>(node, "lookahead_curvature_gain_m");
  config.pure_pursuit.lookahead_min_m =
      requiredParam<double>(node, "lookahead_min_m");
  config.pure_pursuit.lookahead_max_m =
      requiredParam<double>(node, "lookahead_max_m");
  config.pure_pursuit.minimum_target_distance_m =
      requiredParam<double>(node, "minimum_target_distance_m");
  config.pure_pursuit.maximum_steering_angle_rad = degreesToRadians(
      requiredParam<double>(node, "maximum_road_wheel_steering_deg"));

  config.stanley.wheelbase_m = config.wheelbase_m;
  config.stanley.gain = requiredParam<double>(node, "stanley_gain");
  config.stanley.softening_speed_mps =
      requiredParam<double>(node, "stanley_softening_speed_mps");
  config.stanley.minimum_control_speed_mps =
      requiredParam<double>(node, "stanley_minimum_control_speed_mps");
  config.stanley.heading_window_m =
      requiredParam<double>(node, "stanley_heading_window_m");
  config.stanley.heading_error_gain =
      requiredParam<double>(node, "stanley_heading_error_gain");
  config.stanley.curvature_feedforward_gain =
      requiredParam<double>(node, "stanley_curvature_feedforward_gain");
  config.stanley.curvature_preview_distance_m =
      requiredParam<double>(node, "stanley_curvature_preview_distance_m");
  config.stanley.yaw_rate_damping_gain_sec =
      requiredParam<double>(node, "stanley_yaw_rate_damping_gain_sec");
  config.stanley.maximum_steering_angle_rad =
      config.pure_pursuit.maximum_steering_angle_rad;
  config.stanley.maximum_steering_rate_rad_per_sec =
      config.maximum_steering_rate_rad_per_sec;

  config.command_mapper.road_wheel_angle_at_command_limit_rad =
      degreesToRadians(requiredParam<double>(
          node, "road_wheel_angle_at_command_limit_deg"));
  const int command_limit_deg =
      requiredParam<int>(node, "steering_command_limit_deg");
  if (command_limit_deg <= 0 ||
      command_limit_deg > std::numeric_limits<int16_t>::max()) {
    throw std::runtime_error("invalid steering_command_limit_deg");
  }
  config.command_mapper.command_limit_deg =
      static_cast<int16_t>(command_limit_deg);
  config.command_mapper.steering_command_sign =
      requiredParam<int>(node, "steering_command_sign");

  if (config.controller_mode != "pure_pursuit" &&
      config.controller_mode != "stanley") {
    throw std::runtime_error(
        "lateral_controller must be pure_pursuit or stanley");
  }
  if (!positiveFinite(config.control_rate_hz) ||
      !positiveFinite(config.path_timeout_sec) ||
      !positiveFinite(config.mission_timeout_sec) ||
      !positiveFinite(config.odometry_timeout_sec) ||
      !positiveFinite(config.feedback_timeout_sec) ||
      !positiveFinite(config.maximum_control_dt_sec) ||
      !positiveFinite(config.curvature_preview_distance_m) ||
      !positiveFinite(config.maximum_steering_rate_rad_per_sec) ||
      !std::isfinite(config.rear_axle_to_pose_reference_m)) {
    throw std::runtime_error("invalid control timing or geometry parameter");
  }
  if (!config.calibration_required &&
      (!positiveFinite(config.wheelbase_m) ||
       !isValidT870SteeringCommandConfig(config.command_mapper))) {
    throw std::runtime_error(
        "T870 geometry must be valid before calibration_required is false");
  }
  return config;
}

double maximumPreviewCurvature(const std::vector<Point2d>& path,
                               double preview_distance_m) {
  if (path.size() < 3U) {
    return 0.0;
  }
  double maximum_curvature = 0.0;
  double travelled_m = 0.0;
  for (std::size_t index = 1U; index + 1U < path.size(); ++index) {
    travelled_m += std::hypot(path[index].x - path[index - 1U].x,
                              path[index].y - path[index - 1U].y);
    if (travelled_m > preview_distance_m) {
      break;
    }
    const Point2d& first = path[index - 1U];
    const Point2d& second = path[index];
    const Point2d& third = path[index + 1U];
    const double a = std::hypot(second.x - first.x, second.y - first.y);
    const double b = std::hypot(third.x - second.x, third.y - second.y);
    const double c = std::hypot(third.x - first.x, third.y - first.y);
    const double denominator = a * b * c;
    if (!positiveFinite(denominator)) {
      continue;
    }
    const double twice_area =
        2.0 * ((second.x - first.x) * (third.y - first.y) -
               (second.y - first.y) * (third.x - first.x));
    maximum_curvature =
        std::max(maximum_curvature, std::abs(twice_area / denominator));
  }
  return maximum_curvature;
}

}  // namespace

class ControlNode {
 public:
  ControlNode()
      : node_(), private_node_("~"), config_(loadConfig(private_node_)),
        fixed_speed_controller_(config_.fixed_speed),
        stanley_controller_(new StanleyController(config_.stanley)) {
    command_publisher_ =
        node_.advertise<erp42_msgs::DriveCmd>(config_.command_topic, 1);
    state_publisher_ = node_.advertise<std_msgs::String>(config_.state_topic, 1);
    lookahead_publisher_ = node_.advertise<geometry_msgs::PointStamped>(
        config_.lookahead_point_topic, 1);
    stanley_projection_publisher_ =
        node_.advertise<geometry_msgs::PointStamped>(
            config_.stanley_projection_point_topic, 1);
    cross_track_error_publisher_ = node_.advertise<std_msgs::Float64>(
        config_.cross_track_error_topic, 1);
    heading_error_publisher_ = node_.advertise<std_msgs::Float64>(
        config_.heading_error_topic, 1);
    steering_angle_publisher_ = node_.advertise<std_msgs::Float64>(
        config_.steering_angle_topic, 1);

    path_subscriber_ = node_.subscribe(
        config_.path_topic, 1, &ControlNode::onPath, this);
    mission_subscriber_ = node_.subscribe(
        config_.mission_topic, 1, &ControlNode::onMission, this);
    odometry_subscriber_ = node_.subscribe(
        config_.odometry_topic, 1, &ControlNode::onOdometry, this);
    feedback_subscriber_ = node_.subscribe(
        config_.feedback_topic, 1, &ControlNode::onFeedback, this);
    emergency_stop_subscriber_ = node_.subscribe(
        config_.emergency_stop_topic, 1,
        &ControlNode::onEmergencyStop, this);
    timer_ = node_.createWallTimer(
        ros::WallDuration(1.0 / config_.control_rate_hz),
        &ControlNode::onTimer, this);

    if (config_.calibration_required) {
      ROS_WARN("T870 control remains locked: calibration_required=true");
    }
    ROS_INFO("T870 control ready in %s mode", config_.controller_mode.c_str());
  }

 private:
  void onPath(const nav_msgs::Path::ConstPtr& message) {
    latest_path_ = message;
    path_receipt_time_ = ros::SteadyTime::now();
    has_path_ = true;
  }

  void onMission(const planning_interfaces::MissionState::ConstPtr& message) {
    latest_mission_ = message;
    mission_receipt_time_ = ros::SteadyTime::now();
    has_mission_ = true;
  }

  void onOdometry(const nav_msgs::Odometry::ConstPtr& message) {
    latest_odometry_ = message;
    odometry_receipt_time_ = ros::SteadyTime::now();
    has_odometry_ = true;
  }

  void onFeedback(const erp42_msgs::SerialFeedBack::ConstPtr& message) {
    latest_feedback_ = message;
    feedback_receipt_time_ = ros::SteadyTime::now();
    has_feedback_ = true;
  }

  void onEmergencyStop(const std_msgs::Bool::ConstPtr& message) {
    emergency_stop_requested_ = message->data;
  }

  bool fresh(const ros::SteadyTime& now, const ros::SteadyTime& receipt,
             double timeout_sec) const {
    return (now - receipt).toSec() <= timeout_sec;
  }

  bool buildRearAxlePath(std::vector<Point2d>* path,
                         std::string* reason) const {
    if (latest_path_->header.frame_id != config_.expected_frame_id ||
        latest_odometry_->header.frame_id != config_.expected_frame_id ||
        latest_odometry_->child_frame_id != config_.vehicle_frame_id) {
      *reason = "FRAME_MISMATCH";
      return false;
    }
    const geometry_msgs::Point& position = latest_odometry_->pose.pose.position;
    const geometry_msgs::Quaternion& orientation =
        latest_odometry_->pose.pose.orientation;
    const double quaternion_norm =
        std::sqrt(orientation.x * orientation.x +
                  orientation.y * orientation.y +
                  orientation.z * orientation.z +
                  orientation.w * orientation.w);
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !positiveFinite(quaternion_norm) ||
        std::abs(quaternion_norm - 1.0) > 1.0e-3) {
      *reason = "INVALID_POSE";
      return false;
    }

    const double yaw = tf2::getYaw(orientation);
    if (!std::isfinite(yaw)) {
      *reason = "INVALID_POSE_YAW";
      return false;
    }
    const double cosine = std::cos(yaw);
    const double sine = std::sin(yaw);
    path->clear();
    path->reserve(latest_path_->poses.size());
    for (const geometry_msgs::PoseStamped& pose : latest_path_->poses) {
      const double dx = pose.pose.position.x - position.x;
      const double dy = pose.pose.position.y - position.y;
      if (!std::isfinite(dx) || !std::isfinite(dy)) {
        continue;
      }
      path->push_back({
          cosine * dx + sine * dy +
              config_.rear_axle_to_pose_reference_m,
          -sine * dx + cosine * dy,
      });
    }
    if (path->size() < 2U) {
      *reason = "PATH_TOO_SHORT";
      return false;
    }
    return true;
  }

  void publishState(const std::string& state) {
    std_msgs::String message;
    message.data = state;
    state_publisher_.publish(message);
  }

  void publishSafe(const std::string& reason, bool emergency_stop = false) {
    erp42_msgs::DriveCmd command;
    command.KPH = 0U;
    command.Deg = 0;
    command.brake = 1U;
    command.Gear = erp42_msgs::DriveCmd::GEAR_NEUTRAL;
    command.EStop = emergency_stop ? 1U : 0U;
    command_publisher_.publish(command);
    previous_steering_angle_rad_ = 0.0;
    publishState(reason);
    ROS_WARN_THROTTLE(1.0, "T870 safe stop: %s", reason.c_str());
  }

  void publishPoint(const Point2d& rear_axle_point,
                    const ros::Publisher& publisher) const {
    geometry_msgs::PointStamped message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = config_.vehicle_frame_id;
    message.point.x =
        rear_axle_point.x - config_.rear_axle_to_pose_reference_m;
    message.point.y = rear_axle_point.y;
    message.point.z = 0.0;
    publisher.publish(message);
  }

  void onTimer(const ros::WallTimerEvent&) {
    const ros::SteadyTime now = ros::SteadyTime::now();
    double dt_sec = 1.0 / config_.control_rate_hz;
    if (has_timer_time_) {
      dt_sec = (now - last_timer_time_).toSec();
    }
    last_timer_time_ = now;
    has_timer_time_ = true;
    if (emergency_stop_requested_) {
      publishSafe("EMERGENCY_STOP_REQUESTED", true);
      return;
    }
    // A mission E-Stop stays asserted even if its message later becomes stale.
    // Only a fresh MissionState with this bit cleared may release it.
    if (has_mission_ && latest_mission_->emergency_stop_requested) {
      publishSafe("MISSION_EMERGENCY_STOP_REQUESTED", true);
      return;
    }
    if (!positiveFinite(dt_sec) || dt_sec > config_.maximum_control_dt_sec) {
      publishSafe("INVALID_CONTROL_DT");
      return;
    }
    if (config_.calibration_required) {
      publishSafe("CALIBRATION_REQUIRED");
      return;
    }
    if (!has_path_ || !fresh(now, path_receipt_time_,
                             config_.path_timeout_sec)) {
      publishSafe("STALE_PATH");
      return;
    }
    if (!has_mission_ || !fresh(now, mission_receipt_time_,
                                config_.mission_timeout_sec)) {
      publishSafe("STALE_MISSION");
      return;
    }
    if (!latest_mission_->valid || latest_mission_->finished ||
        latest_mission_->stop_requested) {
      const std::string reason =
          latest_mission_->finished
              ? "MISSION_FINISHED"
              : (latest_mission_->stop_requested ? "MISSION_STOP_REQUESTED"
                                                  : "MISSION_INVALID");
      publishSafe(reason);
      return;
    }
    if (latest_mission_->direction != 1 && latest_mission_->direction != -1) {
      publishSafe("MISSION_DIRECTION_INVALID");
      return;
    }
    if (!positiveFinite(latest_mission_->speed_limit_mps)) {
      publishSafe("MISSION_SPEED_LIMIT_INVALID");
      return;
    }
    if (!has_odometry_ || !fresh(now, odometry_receipt_time_,
                                 config_.odometry_timeout_sec)) {
      publishSafe("STALE_ODOMETRY");
      return;
    }
    if (!has_feedback_ || !fresh(now, feedback_receipt_time_,
                                 config_.feedback_timeout_sec)) {
      publishSafe("STALE_FEEDBACK");
      return;
    }
    if (latest_feedback_->EStop != 0U) {
      // Do not echo feedback EStop back into the command. Otherwise a cleared
      // external request can latch itself through the Arduino feedback loop.
      // Keep the vehicle stopped while allowing the command-side EStop to
      // clear; a still-active RC emergency stop remains visible in feedback.
      publishSafe("ESTOP_ACTIVE");
      return;
    }
    if (config_.require_ros_mode && latest_feedback_->MorA != 1U) {
      publishSafe("NOT_IN_ROS_MODE");
      return;
    }
    if (!std::isfinite(latest_feedback_->speed)) {
      publishSafe("INVALID_SPEED_FEEDBACK");
      return;
    }
    std::vector<Point2d> rear_axle_path;
    std::string invalid_reason;
    if (!buildRearAxlePath(&rear_axle_path, &invalid_reason)) {
      publishSafe(invalid_reason);
      return;
    }
    if (latest_mission_->direction < 0) {
      if (config_.controller_mode != "pure_pursuit") {
        publishSafe("REVERSE_REQUIRES_PURE_PURSUIT");
        return;
      }
      // Pure Pursuit selects targets in front of its tracking direction.
      // Reflect longitudinal coordinates so a path behind the body can be
      // tracked while retaining the physical steering sign for reverse.
      for (Point2d& point : rear_axle_path) {
        point.x = -point.x;
      }
    }
    // Gear selects travel direction; lateral controllers need speed magnitude.
    const double speed_mps = std::abs(latest_feedback_->speed);

    double requested_steering_angle_rad = 0.0;
    PurePursuitResult pure_pursuit;
    StanleyResult stanley;
    if (config_.controller_mode == "pure_pursuit") {
      try {
        const double preview_curvature_m_inv = maximumPreviewCurvature(
            rear_axle_path, config_.curvature_preview_distance_m);
        pure_pursuit = computePurePursuit(
            rear_axle_path, speed_mps, preview_curvature_m_inv,
            config_.pure_pursuit);
      } catch (const std::exception& error) {
        ROS_WARN_THROTTLE(1.0, "Pure Pursuit exception: %s", error.what());
        publishSafe("PURE_PURSUIT_EXCEPTION");
        return;
      }
      if (!pure_pursuit.valid) {
        publishSafe("INVALID_PURE_PURSUIT");
        return;
      }
      requested_steering_angle_rad = pure_pursuit.steering_angle_rad;
    } else {
      stanley = stanley_controller_->calculate(
          rear_axle_path, speed_mps, 0.0,
          previous_steering_angle_rad_, dt_sec);
      if (!stanley.valid) {
        publishSafe("INVALID_STANLEY");
        return;
      }
      requested_steering_angle_rad = stanley.requested_steering_angle_rad;
    }

    const double maximum_change_rad =
        config_.maximum_steering_rate_rad_per_sec * dt_sec;
    const double steering_angle_rad = clamp(
        requested_steering_angle_rad,
        previous_steering_angle_rad_ - maximum_change_rad,
        previous_steering_angle_rad_ + maximum_change_rad);
    int16_t steering_command_deg = 0;
    try {
      steering_command_deg = roadWheelAngleToT870CommandDeg(
          steering_angle_rad, config_.command_mapper);
    } catch (const std::exception&) {
      publishSafe("INVALID_STEERING_COMMAND_MAPPING");
      return;
    }

    erp42_msgs::DriveCmd command;
    const double limited_kph = latest_mission_->speed_limit_mps * 3.6;
    const uint16_t mission_limit_kph = static_cast<uint16_t>(std::floor(
        std::min(limited_kph,
                 static_cast<double>(std::numeric_limits<uint16_t>::max()))));
    uint16_t target_speed_kph = fixed_speed_controller_.commandKph();
    if (latest_mission_->mission == "T_PARKING") {
      target_speed_kph = config_.parking_target_speed_kph;
    } else if (latest_mission_->mission == "PARALLEL_PARKING") {
      target_speed_kph = config_.parallel_parking_target_speed_kph;
    }
    command.KPH = std::min(target_speed_kph, mission_limit_kph);
    if (command.KPH == 0U) {
      publishSafe("MISSION_SPEED_LIMIT_ZERO");
      return;
    }
    command.Deg = steering_command_deg;
    command.brake = 0U;
    command.Gear = latest_mission_->direction > 0
                       ? erp42_msgs::DriveCmd::GEAR_FORWARD
                       : erp42_msgs::DriveCmd::GEAR_REVERSE;
    command.EStop = 0U;
    command_publisher_.publish(command);
    previous_steering_angle_rad_ = steering_angle_rad;
    publishState("ACTIVE_" + config_.controller_mode);

    if (pure_pursuit.valid) {
      publishPoint(pure_pursuit.target, lookahead_publisher_);
    }
    if (stanley.valid) {
      publishPoint(stanley.target, stanley_projection_publisher_);
      std_msgs::Float64 cross_track_error;
      cross_track_error.data = stanley.cross_track_error_m;
      cross_track_error_publisher_.publish(cross_track_error);
      std_msgs::Float64 heading_error;
      heading_error.data = stanley.heading_error_rad;
      heading_error_publisher_.publish(heading_error);
    }
    std_msgs::Float64 steering_angle;
    steering_angle.data = steering_angle_rad;
    steering_angle_publisher_.publish(steering_angle);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  NodeConfig config_;
  FixedSpeedController fixed_speed_controller_;
  std::unique_ptr<StanleyController> stanley_controller_;
  ros::Publisher command_publisher_;
  ros::Publisher state_publisher_;
  ros::Publisher lookahead_publisher_;
  ros::Publisher stanley_projection_publisher_;
  ros::Publisher cross_track_error_publisher_;
  ros::Publisher heading_error_publisher_;
  ros::Publisher steering_angle_publisher_;
  ros::Subscriber path_subscriber_;
  ros::Subscriber mission_subscriber_;
  ros::Subscriber odometry_subscriber_;
  ros::Subscriber feedback_subscriber_;
  ros::Subscriber emergency_stop_subscriber_;
  ros::WallTimer timer_;
  nav_msgs::Path::ConstPtr latest_path_;
  planning_interfaces::MissionState::ConstPtr latest_mission_;
  nav_msgs::Odometry::ConstPtr latest_odometry_;
  erp42_msgs::SerialFeedBack::ConstPtr latest_feedback_;
  ros::SteadyTime path_receipt_time_;
  ros::SteadyTime mission_receipt_time_;
  ros::SteadyTime odometry_receipt_time_;
  ros::SteadyTime feedback_receipt_time_;
  ros::SteadyTime last_timer_time_;
  bool has_path_{false};
  bool has_mission_{false};
  bool has_odometry_{false};
  bool has_feedback_{false};
  bool emergency_stop_requested_{false};
  bool has_timer_time_{false};
  double previous_steering_angle_rad_{0.0};
};

}  // namespace stier_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "control_node");
  try {
    stier_control::ControlNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("failed to start control_node: %s", error.what());
    return 1;
  }
  return 0;
}
