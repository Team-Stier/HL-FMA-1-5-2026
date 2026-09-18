#pragma once
#include <stdint.h>

// ROS gear direction is independent of speed magnitude. Encoder observations
// are counted once per fresh sample, including normal neutral/brake holds.
namespace StierRosDrive {
constexpr uint8_t kForward = 0;
constexpr uint8_t kNeutral = 1;
constexpr uint8_t kReverse = 2;
struct DirectionState {
  int8_t accepted;
  uint8_t stationarySamples;
};
inline int8_t direction(float targetKph) {
  return targetKph > 0.f ? 1 : targetKph < 0.f ? -1 : 0;
}
inline void observe(DirectionState& state, bool freshEncoder, long delta,
                    float speedMps, int16_t frontPwm, int16_t rearPwm) {
  if (frontPwm != 0 || rearPwm != 0) state.stationarySamples = 0;
  if (!freshEncoder) return;
  if (delta == 0 && speedMps == 0.f && frontPwm == 0 && rearPwm == 0) {
    if (state.stationarySamples < 3) ++state.stationarySamples;
  } else state.stationarySamples = 0;
}
inline bool select(DirectionState& state, int8_t requested) {
  if (requested == 0) return false;
  if (requested != state.accepted) {
    if (state.stationarySamples < 3) return false;
    state.accepted = requested;
  }
  return true;
}
inline uint8_t gear(int8_t direction) {
  return direction > 0 ? kForward : direction < 0 ? kReverse : kNeutral;
}
}  // namespace StierRosDrive
