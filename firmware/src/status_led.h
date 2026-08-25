#pragma once

#include <Arduino.h>
#include <stdint.h>
#include "hardware/gpio.h"

class StatusLed {
public:
  StatusLed(
    uint8_t pin,
    uint32_t startupBlinkUs,
    uint32_t timeoutBlinkUs,
    uint32_t controlledBlinkUs
  ) : pin_(pin),
      startupBlinkUs_(startupBlinkUs),
      timeoutBlinkUs_(timeoutBlinkUs),
      controlledBlinkUs_(controlledBlinkUs) {}

  void begin() {
    pinMode(pin_, OUTPUT);
    level_ = false;
    lastToggleUs_ = micros();
    gpio_put(pin_, false);
  }

  void serviceStartup() {
    serviceAt(micros(), startupBlinkUs_);
  }

  void serviceInterface() {
    const uint32_t nowUs = micros();
    if (!normalStarted_) {
      serviceAt(nowUs, startupBlinkUs_);
    } else if (pcControlActive(nowUs)) {
      serviceAt(nowUs, controlledBlinkUs_);
    } else {
      serviceAt(nowUs, timeoutBlinkUs_);
    }
  }

  void noteNormalStarted() {
    normalStarted_ = true;
  }

  void noteCommand(uint32_t nowUs, uint16_t timeoutMs, bool anyMotorCommand) {
    if (timeoutMs > 0 && anyMotorCommand) {
      controlDeadlineUs_ = nowUs + (uint32_t)timeoutMs * 1000u;
    } else {
      controlDeadlineUs_ = 0;
    }
  }

  void delayStartup(uint32_t durationMs) {
    const uint32_t startMs = millis();
    while ((millis() - startMs) < durationMs) {
      serviceStartup();
      delay(1);
    }
  }

private:
  void serviceAt(uint32_t nowUs, uint32_t intervalUs) {
    if ((nowUs - lastToggleUs_) < intervalUs) {
      return;
    }

    lastToggleUs_ = nowUs;
    level_ = !level_;
    gpio_put(pin_, level_);
  }

  bool pcControlActive(uint32_t nowUs) const {
    if (controlDeadlineUs_ == 0) {
      return false;
    }
    return (int32_t)(controlDeadlineUs_ - nowUs) > 0;
  }

  const uint8_t pin_;
  const uint32_t startupBlinkUs_;
  const uint32_t timeoutBlinkUs_;
  const uint32_t controlledBlinkUs_;
  bool level_ = false;
  bool normalStarted_ = false;
  uint32_t controlDeadlineUs_ = 0;
  uint32_t lastToggleUs_ = 0;
};
