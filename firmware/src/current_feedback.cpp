#include "current_feedback.h"

#include "board_config.h"
#include "control_math.h"

BU79100QuadReader currentAdc(pio0, GPIO_ADC_SCK, GPIO_ADC_CSB, GPIO_M0_ADC_DATA_A, GPIO_ADC_SYNC_PWM);
BoardCurrentSense currentSense0(0);
BoardCurrentSense currentSense1(1);

namespace {
BU79100QuadSample sample;
uint32_t observedUs;
bool frozen = false;
bool valid = false;

PhaseCurrent_s phaseCurrents(uint8_t motor) {
  const float a = (float(sample.raw[2 * motor]) - ADC_ZERO_CURRENT_COUNTS) * ADC_COUNT_TO_PHASE_CURRENT_A;
  const float c = (float(sample.raw[2 * motor + 1]) - ADC_ZERO_CURRENT_COUNTS) * ADC_COUNT_TO_PHASE_CURRENT_A;
  const float ia = 1.025489f * a + 0.011936f * c;
  const float ic = 0.022043f * a + 0.996548f * c;
  return {ia, -ia - ic, ic};
}
}

bool current_feedback::refresh() {
  const auto next = currentAdc.read();
  if (!next.valid) {
    valid = false;
    return false;
  }
  if (sample.valid && next.sequence == sample.sequence) return false;
  sample = next;
  observedUs = micros();
  valid = true;
  for (auto raw : sample.raw)
    valid = valid && raw > CURRENT_ADC_RAIL_MARGIN && raw < 4095 - CURRENT_ADC_RAIL_MARGIN;
  return valid;
}

bool current_feedback::waitNext(uint32_t timeoutUs) {
  const uint32_t start = micros();
  while (!control_math::expired(micros(), start, timeoutUs)) {
    if (refresh()) return true;
    delayMicroseconds(1);
  }
  return false;
}

bool current_feedback::healthy(uint32_t nowUs) {
  return valid && !control_math::expired(nowUs, observedUs, CURRENT_FEEDBACK_TIMEOUT_US);
}

uint32_t current_feedback::sequence() { return sample.sequence; }
void current_feedback::freeze(bool enabled) { frozen = enabled; }

int BoardCurrentSense::init() {
  initialized = false;
  offset_ia = offset_ib = offset_ic = 0.0f;
  for (uint16_t i = 0; i < CURRENT_SENSE_CALIBRATION_SAMPLES; ++i) {
    if (!current_feedback::waitNext(CURRENT_FEEDBACK_STARTUP_TIMEOUT_US)) return 0;
    const auto phase = phaseCurrents(motor_);
    // Reject disconnected/railed ADCs before offset subtraction can hide them.
    if (fabsf(phase.a) > CURRENT_OFFSET_LIMIT_A || fabsf(phase.b) > CURRENT_OFFSET_LIMIT_A ||
        fabsf(phase.c) > CURRENT_OFFSET_LIMIT_A) return 0;
    offset_ia += phase.a;
    offset_ib += phase.b;
    offset_ic += phase.c;
  }
  offset_ia /= CURRENT_SENSE_CALIBRATION_SAMPLES;
  offset_ib /= CURRENT_SENSE_CALIBRATION_SAMPLES;
  offset_ic /= CURRENT_SENSE_CALIBRATION_SAMPLES;
  initialized = true;
  return 1;
}

PhaseCurrent_s BoardCurrentSense::getPhaseCurrents() {
  if (!frozen) current_feedback::refresh();
  if (!initialized || !current_feedback::healthy(micros())) return {NAN, NAN, NAN};
  auto phase = phaseCurrents(motor_);
  phase.a -= offset_ia;
  phase.b -= offset_ib;
  phase.c -= offset_ic;
  lastCurrent_ = phase;
  return phase;
}
