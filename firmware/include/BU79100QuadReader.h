#pragma once

#include <Arduino.h>
#include "hardware/pio.h"

struct BU79100QuadSample {
  uint16_t raw[4] = {0, 0, 0, 0};
  uint32_t word0 = 0;
  uint32_t word1 = 0;
};

class BU79100QuadReader {
 public:
  BU79100QuadReader(PIO pio, uint8_t pinSck, uint8_t pinCsb, uint8_t pinD0, uint8_t pinTrigger);

  bool init(float sckHz);
  BU79100QuadSample read() const;

 private:
  static constexpr uint32_t RING_WORDS = 64;
  static constexpr uint32_t RING_BYTES = RING_WORDS * sizeof(uint32_t);

  static void appendNibble(uint32_t nibble, uint16_t raw[4]);

  PIO pio_;
  uint8_t pinSck_;
  uint8_t pinCsb_;
  uint8_t pinD0_;
  uint8_t pinTrigger_;
  int sm_ = -1;
  int dmaA_ = -1;
  int dmaB_ = -1;
  alignas(RING_BYTES) volatile uint32_t buffer_[RING_WORDS] = {};
  alignas(4) volatile uint32_t reloadCount_ = RING_WORDS;
};
