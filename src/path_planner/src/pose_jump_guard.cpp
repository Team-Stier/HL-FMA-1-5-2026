#include "path_planner/pose_jump_guard.hpp"

#include <cmath>

namespace path_planner {

PoseJumpGuard::PoseJumpGuard(const PoseJumpGuardConfig& config)
    : config_(config) {}

bool PoseJumpGuard::updatePose(const Pose2d& pose, double stamp_sec,
                               std::string* reason) {
  if (!pose_initialized_) {
    previous_pose_ = pose;
    previous_pose_stamp_sec_ = stamp_sec;
    pose_initialized_ = true;
    return false;
  }

  const double interval = stamp_sec - previous_pose_stamp_sec_;
  const double translation =
      std::hypot(pose.x - previous_pose_.x, pose.y - previous_pose_.y);
  const double yaw_change =
      std::abs(normalizeAngle(pose.yaw - previous_pose_.yaw));
  previous_pose_ = pose;
  previous_pose_stamp_sec_ = stamp_sec;

  if (!std::isfinite(interval) || interval <= 0.0) {
    if (translation > 1.0e-6 || yaw_change > 1.0e-6) {
      if (reason != nullptr) {
        *reason = "POSE_TIMESTAMP_DISCONTINUITY";
      }
      return true;
    }
    return false;
  }
  if (interval > config_.maximum_pose_interval_sec) {
    if (reason != nullptr) {
      *reason = "POSE_TIME_GAP";
    }
    return true;
  }
  const double allowed_translation =
      config_.translation_tolerance_m + config_.maximum_speed_mps * interval;
  if (translation > allowed_translation) {
    if (reason != nullptr) {
      *reason = "POSE_TRANSLATION_JUMP";
    }
    return true;
  }
  const double allowed_yaw = config_.yaw_tolerance_rad +
                             config_.maximum_yaw_rate_rad_per_sec * interval;
  if (yaw_change > allowed_yaw) {
    if (reason != nullptr) {
      *reason = "POSE_YAW_JUMP";
    }
    return true;
  }
  return false;
}

bool PoseJumpGuard::updateMapToOdom(const Pose2d& transform,
                                    std::string* reason) {
  if (!map_odom_initialized_) {
    previous_map_to_odom_ = transform;
    map_odom_initialized_ = true;
    return false;
  }
  const double translation = std::hypot(
      transform.x - previous_map_to_odom_.x,
      transform.y - previous_map_to_odom_.y);
  const double yaw_change = std::abs(
      normalizeAngle(transform.yaw - previous_map_to_odom_.yaw));
  previous_map_to_odom_ = transform;
  if (translation > config_.map_odom_translation_threshold_m) {
    if (reason != nullptr) {
      *reason = "MAP_ODOM_TRANSLATION_JUMP";
    }
    return true;
  }
  if (yaw_change > config_.map_odom_yaw_threshold_rad) {
    if (reason != nullptr) {
      *reason = "MAP_ODOM_YAW_JUMP";
    }
    return true;
  }
  return false;
}

void PoseJumpGuard::reset() {
  pose_initialized_ = false;
  map_odom_initialized_ = false;
}

}  // namespace path_planner
