#pragma once

#include <cstdint>

namespace emergency_stop {
void init();
void poll(); // no interrupt; safe to call from either core
bool active();
void applyOutputInhibit(); // reapply after configuring the PWM pin functions
bool resetRequested(uint32_t &pressGeneration);
bool releaseInhibit(uint32_t pressGeneration);
}
