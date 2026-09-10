#include "board_pwm.h"

#include "board_config.h"
#include "control_math.h"
#include "emergency_stop.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"

namespace {
uint16_t top;
uint16_t guard;
uint16_t levels[6] = {};
bool batched = false;
bool running = false;
bool finiteFrame = true;
uint32_t frameStartUs;
uint32_t frameDurationUs;
uint32_t clockHz;
float inverseTop;

bool commit(bool requireDownCount = false) {
  // At least guard ticks remain before zero, in either count direction.
  // Three packed writes also avoid read/modify/write of the shared M0C/M1A slice.
  const uint32_t irq = save_and_disable_interrupts();
  const uint16_t before = pwm_get_counter(0);
  const uint16_t after = pwm_get_counter(0);
  const bool ok = !running || (requireDownCount ?
    control_math::beforePwmLatch(before, after, guard) : after >= guard);
  if (ok) {
    pwm_set_both_levels(0, levels[0], levels[1]);
    pwm_set_both_levels(1, levels[2], levels[3]);
    pwm_set_both_levels(2, levels[4], levels[5]);
  }
  restore_interrupts(irq);
  return ok;
}

void commitBlocking() {
  // Only calibration/board-test calls use this path. At most 2 * guard ticks.
  while (!commit()) tight_loop_contents();
}
}

void board_pwm::init() {
  static_assert(GPIO_M0_PWM_A == 0 && GPIO_M0_PWM_B == 1 && GPIO_M0_PWM_C == 2 &&
                GPIO_M1_PWM_A == 3 && GPIO_M1_PWM_B == 4 && GPIO_M1_PWM_C == 5,
                "Board PWM layout changed");
  clockHz = clock_get_hz(clk_sys);
  // Phase-correct period is 2 * TOP, not 2 * (TOP + 1).
  top = clockHz / (2 * PWM_FREQUENCY);
  inverseTop = 1.0f / float(top);
  guard = (clockHz / 1000000u) * PWM_COMMIT_GUARD_US;
  pwm_config config = pwm_get_default_config();
  pwm_config_set_phase_correct(&config, true);
  pwm_config_set_wrap(&config, top);
  const uint triggerSlice = pwm_gpio_to_slice_num(GPIO_ADC_SYNC_PWM);
  for (uint slice : {0u, 1u, 2u, triggerSlice}) pwm_init(slice, &config, false);
  for (uint pin = 0; pin < 6; ++pin) gpio_set_function(pin, GPIO_FUNC_PWM);
  emergency_stop::applyOutputInhibit();
  gpio_set_function(GPIO_ADC_SYNC_PWM, GPIO_FUNC_PWM);
  pwm_set_gpio_level(GPIO_ADC_SYNC_PWM, uint16_t(top * ADC_TRIGGER_DUTY));
  running = false;
  stop();
}

void board_pwm::start() {
  const uint triggerSlice = pwm_gpio_to_slice_num(GPIO_ADC_SYNC_PWM);
  for (uint slice : {0u, 1u, 2u, triggerSlice}) pwm_set_counter(slice, 0);
  pwm_set_mask_enabled(7u | (1u << triggerSlice));
  running = true;
}

void board_pwm::beginFrame() {
  batched = true;
  finiteFrame = true;
  frameStartUs = micros();
}

bool board_pwm::finishFrame() {
  batched = false;
  frameDurationUs = micros() - frameStartUs;
  return finiteFrame && frameDurationUs < CONTROL_PERIOD_US - PWM_COMMIT_GUARD_US;
}

bool board_pwm::commitStagedFrame() { return commit(true); }

bool board_pwm::commitFrame() {
  // The inductance test applies its voltage at the immediately following zero.
  // Its short calculation must fit this half-cycle, unlike pipelined FOC.
  return finishFrame() && frameDurationUs < CONTROL_PERIOD_US / 2 && commit(true);
}

uint32_t board_pwm::lastFrameDurationUs() { return frameDurationUs; }

void board_pwm::stop() {
  batched = false;
  for (auto &level : levels) level = 0;
  commitBlocking();
}

float board_pwm::periodUs() { return float(2u * top) * 1.0e6f / float(clockHz); }

float board_pwm::samplePhaseUs() {
  // Real CS falling edge: 42 PIO clocks after the PWM trigger falling edge.
  // This includes the 16-clock dummy frame and the acquisition guard.
  return float(uint16_t(top * ADC_TRIGGER_DUTY)) * 1.0e6f / float(clockHz) +
    42.0f * 1.0e6f / (2.0f * ADC_SCK_HZ);
}

bool board_pwm::samplingWindow() {
  // Decoding and housekeeping can delay observation of a completed ADC frame.
  // Accept it throughout this half-cycle; the compare writes recheck the phase.
  const uint16_t before = pwm_get_counter(0);
  const uint16_t after = pwm_get_counter(0);
  return running && control_math::beforePwmLatch(before, after, guard);
}

void BoardMotorDriver::init(SPIClass *spi) {
  DRV8316Driver::init(spi);
  setRegistersLocked(false);
  setPWMMode(DRV8316_PWMMode::PWM3_Mode);
  updateSupplyVoltage(voltage_power_supply);
  initialized = true;
}

void BoardMotorDriver::updateSupplyVoltage(float supply) {
  voltage_power_supply = supply;
  countsPerVolt_ = supply > 0.0f ? float(top) / supply : 0.0f;
}

void BoardMotorDriver::setPwm(float ua, float ub, float uc) {
  if (!initialized) return;
  emergency_stop::poll();
  if (emergency_stop::active()) ua = ub = uc = 0.0f;
  if (!isfinite(ua) || !isfinite(ub) || !isfinite(uc)) {
    finiteFrame = false;
    ua = ub = uc = 0.0f;
  }
  const bool zero = ua == 0.0f && ub == 0.0f && uc == 0.0f;
  const float volts[3] = {ua, ub, uc};
  float *duties[3] = {&dc_a, &dc_b, &dc_c};
  for (uint i = 0; i < 3; ++i) {
    const float count = zero ? 0.0f : _constrain(
      _constrain(volts[i], 0.0f, voltage_limit) * countsPerVolt_,
      top * MOTOR_PWM_ACTIVE_MIN_DUTY, top * MOTOR_PWM_ACTIVE_MAX_DUTY);
    levels[motor_ * 3 + i] = uint16_t(count);
    *duties[i] = count * inverseTop;
  }
  if (!batched) commitBlocking();
}
