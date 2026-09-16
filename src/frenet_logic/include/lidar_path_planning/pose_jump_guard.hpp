#pragma once

#include <string>

#include "lidar_path_planning/types.hpp"

namespace lidar_path_planning {

struct PoseJumpGuardConfig {
  double translation_tolerance_m{0.25};
  double yaw_tolerance_rad{0.08726646259971647};
  double maximum_speed_mps{5.0};
  double maximum_yaw_rate_rad_per_sec{2.0943951023931953};
  double maximum_pose_interval_sec{0.50};
  double map_odom_translation_threshold_m{0.20};
  double map_odom_yaw_threshold_rad{0.05235987755982989};
};

class PoseJumpGuard {
 public:
  explicit PoseJumpGuard(const PoseJumpGuardConfig& config);

  // Returns true when the new sample is discontinuous. The new sample still
  // becomes the baseline so one correction produces only one invalidation.
  bool updatePose(const Pose2d& pose, double stamp_sec, std::string* reason);
  bool updateMapToOdom(const Pose2d& transform, std::string* reason);
  void reset();

 private:
  PoseJumpGuardConfig config_;
  bool pose_initialized_{false};
  Pose2d previous_pose_;
  double previous_pose_stamp_sec_{0.0};
  bool map_odom_initialized_{false};
  Pose2d previous_map_to_odom_;
};

}  // namespace lidar_path_planning
