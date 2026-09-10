#pragma once

#include <SimpleFOC.h>
#include "BU79100QuadReader.h"

extern BU79100QuadReader currentAdc;

namespace current_feedback {
bool refresh(); // true only for a new, valid frame
bool waitNext(uint32_t timeoutUs);
bool healthy(uint32_t nowUs);
uint32_t sequence();
void freeze(bool enabled); // both FOC calls consume the same frame
}

class BoardCurrentSense : public CurrentSense {
public:
  explicit BoardCurrentSense(uint8_t motor) : motor_(motor) {}
  int init() override;
  PhaseCurrent_s getPhaseCurrents() override;
  PhaseCurrent_s lastPhaseCurrents() const { return lastCurrent_; }
  int driverAlign(float, bool = false) override { return initialized ? 1 : 0; }
private:
  uint8_t motor_;
  PhaseCurrent_s lastCurrent_ = {};
};

extern BoardCurrentSense currentSense0;
extern BoardCurrentSense currentSense1;
