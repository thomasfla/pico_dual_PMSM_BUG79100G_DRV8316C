#include "emergency_stop.h"
#include "emergency_button.h"
#include "board_config.h"
#include "hardware/gpio.h"
#include <atomic>

namespace {
std::atomic<bool> stopped{false}, wasPressed{false};
std::atomic<uint32_t> generation{0};
std::atomic_flag transitionLock = ATOMIC_FLAG_INIT;
EmergencyButton button(ESTOP_RESET_HOLD_US, ESTOP_RELEASE_DEBOUNCE_US);
uint32_t servicedGeneration = 0;

void lock() { while (transitionLock.test_and_set(std::memory_order_acquire)) {} }
void unlock() { transitionLock.clear(std::memory_order_release); }

void overrideOutputs(bool inhibit) {
  for (uint pin = GPIO_M0_PWM_A; pin <= GPIO_M1_PWM_C; ++pin)
    gpio_set_outover(pin, inhibit ? GPIO_OVERRIDE_LOW : GPIO_OVERRIDE_NORMAL);
}

}

void emergency_stop::init() {
  pinMode(GPIO_ESTOP, INPUT_PULLUP);
  poll();
}

void emergency_stop::poll() {
  const bool pressedNow = !gpio_get(GPIO_ESTOP);
  if (pressedNow == wasPressed.load(std::memory_order_relaxed)) return;
  // Only transitions take the lock. Core1 also polls during blocking boot tools.
  lock();
  const bool pressed = !gpio_get(GPIO_ESTOP);
  if (pressed != wasPressed.load(std::memory_order_relaxed)) {
    wasPressed.store(pressed, std::memory_order_relaxed);
    if (pressed) {
      generation.fetch_add(1, std::memory_order_relaxed);
      stopped.store(true, std::memory_order_relaxed);
      overrideOutputs(true);
    }
  }
  unlock();
}

bool emergency_stop::active() { return stopped.load(std::memory_order_relaxed); }

void emergency_stop::applyOutputInhibit() {
  lock();
  overrideOutputs(active());
  unlock();
}

bool emergency_stop::resetRequested(uint32_t &pressGeneration) {
  if (!active()) return false;
  const uint32_t now = micros();
  pressGeneration = generation.load(std::memory_order_relaxed);
  if (pressGeneration != servicedGeneration) {
    button.notePress(now);
    servicedGeneration = pressGeneration;
  }
  return button.update(!gpio_get(GPIO_ESTOP), now);
}

bool emergency_stop::releaseInhibit(uint32_t pressGeneration) {
  lock();
  // The qualified reset press may still be held. Reject a new press or an
  // input transition that has not been polled yet during feedback validation.
  const bool ok = generation.load(std::memory_order_relaxed) == pressGeneration &&
    (!gpio_get(GPIO_ESTOP) == wasPressed.load(std::memory_order_relaxed));
  if (ok) {
    button.reset();
    stopped.store(false, std::memory_order_relaxed);
    overrideOutputs(false);
  }
  unlock();
  return ok;
}
