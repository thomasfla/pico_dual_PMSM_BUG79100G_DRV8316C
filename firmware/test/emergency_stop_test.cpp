#include "emergency_stop.h"
#include "emergency_button.h"
#include "board_config.h"
#include "hardware/gpio.h"
#include <cassert>
#include <cstdio>

namespace {
uint32_t now = 0;
bool pinHigh = true;
unsigned overrides[6] = {};
bool pullup = false;
}
uint32_t micros() { return now; }
void pinMode(unsigned pin, uint8_t mode) { assert(pin == 28); pullup = mode == INPUT_PULLUP; }
bool gpio_get(unsigned pin) { assert(pin == 28); return pinHigh; }
void gpio_set_outover(unsigned pin, unsigned value) { assert(pin < 6); overrides[pin] = value; }

static bool sample(bool pressed, uint32_t time, uint32_t &token) {
  pinHigh = !pressed;
  now = time;
  emergency_stop::poll();
  return emergency_stop::resetRequested(token);
}

int main() {
  using namespace emergency_stop;
  init();
  assert(pullup && !active());
  uint32_t token = 0;
  assert(!sample(true, 100, token));
  assert(active());
  for (auto value : overrides) assert(value == GPIO_OVERRIDE_LOW);

  // Holding the initial stop press must never acknowledge the stop.
  assert(!sample(true, 10000100, token));
  assert(active());
  assert(!sample(false, 10000200, token));
  assert(!sample(true, 10010000, token)); // release bounce does not arm reset
  assert(!sample(false, 10020000, token));
  assert(!sample(false, 10040000, token));

  // A short second press is not a reset.
  assert(!sample(true, 10100000, token));
  assert(!sample(false, 13099999, token));
  assert(active());
  assert(!sample(false, 13100000, token));
  assert(!sample(false, 13120000, token));

  assert(!sample(true, 13200000, token));
  assert(!sample(true, 16199999, token));
  assert(sample(true, 16200000, token)); // request reset at 3 s, still held
  assert(active()); // the control loop still has to validate feedback

  // A new press during validation invalidates the reset token.
  pinHigh = true; poll();
  pinHigh = false; poll();
  pinHigh = true; poll();
  assert(!releaseInhibit(token));
  for (auto &value : overrides) value = GPIO_OVERRIDE_NORMAL;
  applyOutputInhibit(); // PWM initialization must preserve a stop asserted at boot
  for (auto value : overrides) assert(value == GPIO_OVERRIDE_LOW);

  assert(!sample(false, 17000000, token));
  assert(!sample(false, 17020000, token));
  assert(!sample(true, 17100000, token));
  assert(sample(true, 20100000, token));
  assert(releaseInhibit(token));
  assert(!active());
  for (auto value : overrides) assert(value == GPIO_OVERRIDE_NORMAL);

  // Both the hold and release timers must survive micros() wrapping.
  EmergencyButton wrap(3000000, 20000);
  uint32_t t = UINT32_MAX - 1000000;
  assert(!wrap.update(false, t));
  assert(!wrap.update(false, t += 20000));
  assert(!wrap.update(true, t += 1));
  assert(!wrap.update(true, t += 2999999));
  assert(wrap.update(true, t += 1));

  EmergencyButton betweenPolls(3000000, 20000);
  assert(!betweenPolls.update(false, 0));
  assert(!betweenPolls.update(false, 20000));
  assert(!betweenPolls.update(true, 20001));
  betweenPolls.notePress(2000000); // other core observed an intervening press
  assert(!betweenPolls.update(true, 3020001));
  assert(!betweenPolls.update(false, 4999999)); // still short of a continuous 3 s
  assert(!betweenPolls.update(false, 5020000));
  std::puts("Emergency-stop polling and reset checks passed");
}
