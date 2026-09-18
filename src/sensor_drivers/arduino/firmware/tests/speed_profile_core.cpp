// Hardware-free test, built against each vehicle's actual core and pin stubs.
#include "BroonT870Core.h"
#include <cassert>
#include <cmath>
#include <iostream>

uint32_t testMillis = 0;
uint8_t PINC = 0, PCMSK1 = 0;

int main() {
  using namespace BroonT870;
  SpeedPiState state{};
  // Acceleration still ramps by 1 km/h/s; no new aggressive launch command.
  auto result = updateSpeedPi(state, 15.f, 0.f, 20.f, 8.f, .12f, 1.f, 230, 100);
  assert(std::abs(state.rampedTargetKph - .1f) < 1e-6f);
  state.rampedTargetKph = 15.f;
  state.integralPwm = 80.f;
  result = updateSpeedPi(state, 3.f, 8.f, 20.f, 8.f, .12f, 1.f, 230, 100);
  assert(state.rampedTargetKph == 3.f);
  assert(result.pwm == 0);
  // Signed reverse speed is measured as magnitude, as in the real PI loop.
  state.rampedTargetKph = 5.f;
  state.integralPwm = 30.f;
  result = updateSpeedPi(state, 1.f, -3.f, 20.f, 8.f, .12f, 1.f, 230, 100);
  assert(state.rampedTargetKph == 1.f);
  assert(result.pwm == 0);
  result = updateSpeedPi(state, 0.f, 0.f, 20.f, 8.f, .12f, 1.f, 230, 100);
  assert(state.rampedTargetKph == 0.f && state.integralPwm == 0.f);
  assert(result.pwm == 0);
  std::cout << "PASS: acceleration ramp, immediate lower target, signed speed, zero reset\n";
}
