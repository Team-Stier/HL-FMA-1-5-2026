#include "control/longitudinal/path_speed_profile.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace stier_control {
namespace {
constexpr double kMinimumCommandMps = 1.0 / 3.6;

double distance(const Point2d& a, const Point2d& b) {
  return std::hypot(b.x-a.x, b.y-a.y);
}

std::vector<Point2d> preview(const std::vector<Point2d>& path,
                             const PathSpeedConfig& config) {
  double nearest = std::numeric_limits<double>::infinity();
  Point2d projection;
  std::size_t next = path.size();
  for (std::size_t i=1; i<path.size(); ++i) {
    const auto& a=path[i-1];
    const auto& b=path[i];
    const double dx=b.x-a.x, dy=b.y-a.y, squared=dx*dx+dy*dy;
    if (squared <= 1e-12) continue;
    const double t=std::max(0.0, std::min(1.0, -(a.x*dx+a.y*dy)/squared));
    const Point2d point{a.x+t*dx, a.y+t*dy};
    const double error=std::hypot(point.x, point.y);
    if (error < nearest) { nearest=error; projection=point; next=i; }
  }
  if (next == path.size()) return {};
  std::vector<Point2d> samples{projection};
  Point2d previous=projection;
  double travelled=0.0, next_sample=config.sample_interval_m;
  for (std::size_t i=next; i<path.size(); ++i) {
    const double length=distance(previous, path[i]);
    if (length <= 1e-9) { previous=path[i]; continue; }
    while (next_sample <= travelled+length+1e-9 &&
           next_sample <= config.preview_distance_m+1e-9) {
      const double t=std::max(0.0, std::min(1.0, (next_sample-travelled)/length));
      samples.push_back({previous.x+t*(path[i].x-previous.x),
                         previous.y+t*(path[i].y-previous.y)});
      next_sample+=config.sample_interval_m;
    }
    travelled+=length;
    previous=path[i];
    if (travelled >= config.preview_distance_m) break;
  }
  return samples;
}
}  // namespace

bool isValidPathSpeedConfig(const PathSpeedConfig& c) {
  return std::isfinite(c.wheelbase_m) && c.wheelbase_m>0 &&
      std::isfinite(c.lateral_acceleration_mps2) && c.lateral_acceleration_mps2>0 &&
      std::isfinite(c.preview_deceleration_mps2) && c.preview_deceleration_mps2>0 &&
      std::isfinite(c.reaction_time_sec) && c.reaction_time_sec>=0 &&
      std::isfinite(c.sample_interval_m) && c.sample_interval_m>0 &&
      std::isfinite(c.preview_distance_m) && c.preview_distance_m>=2*c.sample_interval_m;
}

PathSpeedResult computePathSpeed(
    const std::vector<Point2d>& path, double measured_speed_mps,
    double requested_steering_rad, double applied_steering_rad,
    double ceiling_mps, double remaining_stop_m, const PathSpeedConfig& config) {
  PathSpeedResult result;
  result.curve_limit_mps=result.stop_limit_mps=ceiling_mps;
  // Include the requested angle before slew limiting AND the angle actually
  // emitted this cycle. Straightening a command must not instantly lift the cap.
  const double steering_curvature=std::max(std::abs(std::tan(requested_steering_rad)),
      std::abs(std::tan(applied_steering_rad))) / config.wheelbase_m;
  result.maximum_curvature_m_inv=steering_curvature;
  if (steering_curvature>1e-9) {
    result.curve_limit_mps=std::min(result.curve_limit_mps,
        std::sqrt(config.lateral_acceleration_mps2/steering_curvature));
  }
  const double reaction_distance=std::abs(measured_speed_mps)*config.reaction_time_sec;
  const auto samples=preview(path, config);
  const double visible_distance=samples.empty() ? 0.0 :
      (samples.size()-1)*config.sample_interval_m;
  // A short published path is not evidence of a long straight. Keep enough
  // model distance to slow to the wire's minimum moving command at its end.
  // This is a continuous speed cap, never a new brake/stop condition.
  result.visibility_limit_mps=std::min(ceiling_mps,
      std::sqrt(kMinimumCommandMps*kMinimumCommandMps+
          2*config.preview_deceleration_mps2*std::max(0.0, visible_distance-reaction_distance)));
  for (std::size_t i=1; i+1<samples.size(); ++i) {
    const auto& a=samples[i-1]; const auto& b=samples[i]; const auto& c=samples[i+1];
    const double denominator=distance(a,b)*distance(b,c)*distance(a,c);
    if (denominator<=1e-12) continue;
    const double curvature=2*std::abs((b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x))/denominator;
    result.maximum_curvature_m_inv=std::max(result.maximum_curvature_m_inv, curvature);
    if (curvature<=1e-9) continue;
    // v(now)^2 <= v(curve)^2 + 2*a*distance; react before entering the bend.
    const double available=std::max(0.0, i*config.sample_interval_m-reaction_distance);
    const double limit=std::sqrt(config.lateral_acceleration_mps2/curvature+
        2*config.preview_deceleration_mps2*available);
    result.curve_limit_mps=std::min(result.curve_limit_mps, limit);
  }
  if (remaining_stop_m>=0) {
    const double available=std::max(0.0, remaining_stop_m-reaction_distance);
    result.stop_limit_mps=std::min(result.stop_limit_mps,
        std::sqrt(2*config.preview_deceleration_mps2*available));
  }
  // Creeping to an existing mission stop must not round 0.9 km/h down to
  // neutral forever. Never lift the externally supplied mission speed ceiling.
  result.speed_mps=std::min(ceiling_mps, std::max(kMinimumCommandMps,
      std::min(result.visibility_limit_mps, std::min(result.curve_limit_mps, result.stop_limit_mps))));
  return result;
}

}  // namespace stier_control
