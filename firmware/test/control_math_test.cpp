#include "control_math.h"
#include <cassert>
#include <cstdio>
#include <initializer_list>
#include <limits>

static bool near(float a, float b, float tolerance = 1e-5f) {
  return std::fabs(a - b) < tolerance;
}

int main() {
  using namespace control_math;
  // Captured wire order: four dummy bits followed by twelve MSB-first bits
  // on each of the four contiguous ADC data pins. Dummy bits are ignored.
  uint16_t raw[4];
  decodeAdcWords(0xffff9cde, 0xd8b1d92e, raw);
  assert(raw[0] == 0xabc && raw[1] == 0x123 && raw[2] == 0x789 && raw[3] == 0xfed);
  decodeAdcWords(0x00009cde, 0xd8b1d92e, raw);
  assert(raw[0] == 0xabc && raw[1] == 0x123 && raw[2] == 0x789 && raw[3] == 0xfed);
  decodeAdcWords(0, 0, raw);
  for (auto value : raw) assert(value == 0);
  decodeAdcWords(0xffffffff, 0xffffffff, raw);
  for (auto value : raw) assert(value == 4095);

  // Legacy records could contain any number of turns. Rebooting at the
  // calibration point must give zero for either motor direction.
  for (int direction : {-1, 1}) {
    for (int turns = -8; turns <= 8; ++turns) {
      const float saved = direction * (turns * tau + 0.1f);
      const float offset = singleTurnOffset(saved, direction);
      assert(near(direction * 0.1f - offset, 0.0f));
    }
    const float offset = singleTurnOffset(direction * 6.2f, direction);
    const int turns = startupTurns(0.1f, offset, direction);
    const float position = direction * (turns * tau + 0.1f) - offset;
    assert(near(position, direction * (tau + 0.1f - 6.2f)));
  }
  assert(!electricalCalibrationValid(0, 0.5f));
  assert(!electricalCalibrationValid(1, -12345.0f));
  assert(!electricalCalibrationValid(1, std::numeric_limits<float>::quiet_NaN()));
  assert(electricalCalibrationValid(-1, 0.5f));

  // Velocity remains a 1 ms observation at both 20 kHz and 100 kHz input rates.
  for (uint32_t step : {10u, 50u}) {
    VelocityWindow window;
    window.reset(0, 0);
    for (uint32_t t = step; t < 1000; t += step)
      assert(window.update(t, t, 1000, 0.001f) == 0.0f);
    assert(near(window.update(1000, 1000, 1000, 0.001f), 1000.0f));
  }
  VelocityWindow wrap;
  wrap.reset(UINT32_MAX - 499u, 10000);
  assert(near(wrap.update(500, 10010, 1000, 0.001f), 10.0f));
  assert(!expired(50, UINT32_MAX - 49u, 100));
  assert(expired(51, UINT32_MAX - 49u, 100));

  // 150 MHz, TOP=3750, guard=450: a frame observed 6 us after the
  // peak still has 19 us until the latch. The old 4 us window rejected it.
  assert(beforePwmLatch(2852, 2850, 450));
  assert(beforePwmLatch(452, 450, 450));
  assert(!beforePwmLatch(450, 449, 450));
  // A calculation that crosses zero must not commit on the next up-count,
  // even after the counter has climbed above the guard again.
  assert(!beforePwmLatch(600, 602, 450));
  assert(!beforePwmLatch(2850, 2850, 450));

  // A 200 us requested pulse is measured at the first sample after it (225 us),
  // independent of polling/SPI overhead. The first post-latch sample is at 25 us.
  assert(near(stepSampleTimeUs(1, 50, 25), 25));
  assert(near(stepSampleTimeUs(5, 50, 25), 225));
  const float resistance = 3.2f, actualL = 0.0005f, dt = 225e-6f;
  const float current = (1 - std::exp(-resistance * dt / actualL)) / resistance;
  const float estimatedL = -resistance * dt / std::log(1 - resistance * current);
  assert(near(estimatedL, actualL, 1e-8f));
  std::puts("Control regression checks passed");
}
