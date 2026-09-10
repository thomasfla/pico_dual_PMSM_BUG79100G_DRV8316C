// Exercise the production current-feedback/calibration implementation with a
// fake ADC transport and clock; no production test hooks or hardware access.
#include "current_feedback.h"
#include "board_config.h"
#include <cassert>
#include <cstdio>

namespace {
uint32_t now = 0, reads = 0;
BU79100QuadSample incoming;
bool advance = false, skipFirst = false;
}

uint32_t micros() { return now; }
void delayMicroseconds(uint32_t us) { now += us; }

BU79100QuadReader::BU79100QuadReader(PIO pio, uint8_t sck, uint8_t cs,
                                   uint8_t data, uint8_t trigger)
  : pio_(pio), pinSck_(sck), pinCsb_(cs), pinD0_(data), pinTrigger_(trigger) {}

BU79100QuadSample BU79100QuadReader::read() const {
  ++reads;
  if (skipFirst) { skipFirst = false; return {}; }
  if (advance) { now += 50; ++incoming.sequence; }
  return incoming;
}

int main() {
  // No completed frame is an initialization failure, not a zero-current offset.
  assert(currentSense0.init() == 0);
  assert(!currentSense0.initialized);
  assert(!current_feedback::healthy(now));

  incoming.valid = true;
  incoming.sequence = 1;
  for (auto &raw : incoming.raw) raw = 0;
  assert(currentSense0.init() == 0); // disconnected, rail-low ADC

  // Start with one incomplete frame, then valid midscale readings. The invalid
  // frame must not introduce the old ~22 mA offset error.
  for (auto &raw : incoming.raw) raw = 2048;
  advance = true;
  skipFirst = true;
  assert(currentSense0.init() == 1);
  assert(currentSense1.init() == 1);
  assert(currentSense0.offset_ia == 0 && currentSense0.offset_ic == 0);
  assert(currentSense1.offset_ia == 0 && currentSense1.offset_ic == 0);

  incoming.raw[0] = 2100;
  incoming.raw[2] = 2000;
  assert(current_feedback::refresh());
  current_feedback::freeze(true);
  const auto countBefore = reads;
  const auto m0 = currentSense0.getPhaseCurrents();
  const auto m1 = currentSense1.getPhaseCurrents();
  assert(reads == countBefore); // both motors reuse one frame
  assert(m0.a > 0 && m1.a < 0);
  assert(std::fabs(m0.a + m0.b + m0.c) < 1e-6f);

  // A stopped DMA keeps returning the same valid packet. Repeated polling must
  // never refresh its age or keep FOC feedback alive indefinitely.
  advance = false;
  const auto lastGood = now;
  for (unsigned i = 0; i < CURRENT_FEEDBACK_TIMEOUT_US; ++i) {
    ++now;
    assert(!current_feedback::refresh());
    assert(current_feedback::healthy(now));
  }
  ++now;
  assert(!current_feedback::healthy(now));
  assert(std::isnan(currentSense0.getPhaseCurrents().a));
  assert(now - lastGood == CURRENT_FEEDBACK_TIMEOUT_US + 1);

  // Timer wrap must neither trip early nor keep stale feedback alive.
  now = UINT32_MAX - 100;
  ++incoming.sequence;
  assert(current_feedback::refresh());
  assert(current_feedback::healthy(149));
  assert(!current_feedback::healthy(150));

  ++incoming.sequence;
  incoming.raw[0] = 4095;
  assert(!current_feedback::refresh());
  assert(!current_feedback::healthy(now));

  current_feedback::freeze(false);
  incoming.raw[0] = 3000;
  advance = true;
  assert(currentSense0.init() == 0); // excessive offset cannot be calibrated away
  assert(!currentSense0.initialized);
  std::puts("Current feedback fault-injection checks passed");
}
