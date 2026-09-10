#pragma once
#include "Arduino.h"
#define NOT_SET -12345.0f
struct PhaseCurrent_s { float a, b, c; };
class CurrentSense {
public:
  virtual ~CurrentSense() = default;
  virtual int init() = 0;
  virtual PhaseCurrent_s getPhaseCurrents() = 0;
  virtual int driverAlign(float, bool) = 0;
  bool initialized = false;
  float offset_ia = 0, offset_ib = 0, offset_ic = 0;
};
