#pragma once

#include <cstdint>

// The stop edge is never debounced. Reset requires a stable initial release,
// then a separate long press; it fires as soon as the hold time elapses.
class EmergencyButton {
public:
  EmergencyButton(uint32_t holdUs, uint32_t releaseUs)
    : holdUs_(holdUs), releaseUs_(releaseUs) {}

  bool update(bool pressed, uint32_t now) {
    switch (state_) {
      case State::WaitInitialRelease:
        if (released(pressed, now)) state_ = State::Armed;
        break;
      case State::Armed:
        if (pressed) { holdStart_ = now; state_ = State::Holding; }
        break;
      case State::Holding:
        if (!pressed) {
          reset();
        } else if (uint32_t(now - holdStart_) >= holdUs_) {
          reset();
          return true;
        }
        break;
    }
    return false;
  }

  void reset() { state_ = State::WaitInitialRelease; releaseTiming_ = false; }

  void notePress(uint32_t now) {
    // Another polling site may have seen a release/press between updates.
    releaseTiming_ = false;
    if (state_ == State::Armed || state_ == State::Holding) {
      holdStart_ = now;
      state_ = State::Holding;
    }
  }

private:
  enum class State { WaitInitialRelease, Armed, Holding };
  bool released(bool pressed, uint32_t now) {
    if (pressed) { releaseTiming_ = false; return false; }
    if (!releaseTiming_) { releaseStart_ = now; releaseTiming_ = true; }
    return uint32_t(now - releaseStart_) >= releaseUs_;
  }
  const uint32_t holdUs_, releaseUs_;
  State state_ = State::WaitInitialRelease;
  uint32_t holdStart_ = 0, releaseStart_ = 0;
  bool releaseTiming_ = false;
};
