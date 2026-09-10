#pragma once

#include <cmath>
#include <cstdint>

namespace control_math {
constexpr float tau = 6.2831853071795864769f;

inline float singleTurnOffset(float offset, int direction) {
  float raw = std::fmod(offset * direction, tau);
  if (raw < 0.0f) raw += tau;
  return raw * direction;
}

inline bool electricalCalibrationValid(int direction, float angle) {
  return (direction == -1 || direction == 1) &&
    std::isfinite(angle) && angle >= 0.0f && angle < tau;
}

inline int startupTurns(float rawAngle, float offset, int direction) {
  const float delta = rawAngle - offset * direction;
  return delta > tau / 2 ? -1 : (delta < -tau / 2 ? 1 : 0);
}

inline bool expired(uint32_t now, uint32_t last, uint32_t timeout) {
  return uint32_t(now - last) > timeout;
}

inline bool beforePwmLatch(uint16_t before, uint16_t after, uint16_t guard) {
  // Down-counting toward this period's zero, with time for all compare writes.
  return after < before && after >= guard;
}

inline float stepSampleTimeUs(uint32_t frameDelta, float periodUs, float samplePhaseUs) {
  // Voltage is committed after frame N's sample, then latched at the next zero.
  return (float(frameDelta) - 1.0f) * periodUs + samplePhaseUs;
}

inline void decodeAdcWords(uint32_t word0, uint32_t word1, uint16_t raw[4]) {
  for (unsigned channel = 0; channel < 4; ++channel) {
    uint16_t value = 0;
    for (int bit = 3; bit >= 0; --bit)
      value = (value << 1) | ((word0 >> (bit * 4 + channel)) & 1u);
    for (int bit = 7; bit >= 0; --bit)
      value = (value << 1) | ((word1 >> (bit * 4 + channel)) & 1u);
    raw[channel] = value;
  }
}

// Only two reference values; work and window length do not depend on loop rate.
class VelocityWindow {
public:
  void reset(uint32_t now, int64_t count) {
    time_ = now;
    count_ = count;
    value_ = 0.0f;
  }
  float update(uint32_t now, int64_t count, uint32_t windowUs, float radiansPerCount) {
    const uint32_t elapsed = now - time_;
    if (elapsed >= windowUs) {
      value_ = float(count - count_) * radiansPerCount * 1.0e6f / float(elapsed);
      count_ = count;
      time_ = now;
    }
    return value_;
  }
private:
  uint32_t time_ = 0;
  int64_t count_ = 0;
  float value_ = 0.0f;
};
}
