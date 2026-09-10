#pragma once
#include <cstdint>
#include <cstddef>
#include <cmath>
using std::isfinite;
using uint = unsigned;
constexpr uint8_t INPUT_PULLUP = 2;
void pinMode(unsigned pin, uint8_t mode);
uint32_t micros();
void delayMicroseconds(uint32_t us);
