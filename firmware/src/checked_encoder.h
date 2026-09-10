#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/platform.h"
#include "board_config.h"
#include "control_math.h"
#include "shared_spi_bus.h"

struct EncoderRegisterDiagnostics {
  uint16_t magnitude = 0;
  uint16_t diagnostics = 0;
  bool magnitudeOk = false;
  bool diagnosticsOk = false;
};

class CheckedAS5048ASensor : public Sensor {
public:
  explicit CheckedAS5048ASensor(uint8_t csPin)
    : csPin_(csPin) {}

  void init() override {
    angleOk_ = false;
    pinMode(csPin_, OUTPUT);
    digitalWrite(csPin_, HIGH);
    sharedSpiUseEncoder();

    transfer16(makeReadCommand(AS5048A_ANGLE_REG));
    delayMicroseconds(5);
    clearErrorFlag();

    uint16_t raw = 0;
    for (uint8_t i = 0; i < ENCODER_STARTUP_READ_ATTEMPTS; i++) {
      if (readRawChecked(raw)) {
        angleOk_ = true;
        const float angle = rawToAngle(raw);
        angle_prev = angle;
        vel_angle_prev = angle;
        angle_prev_ts = micros();
        vel_angle_prev_ts = angle_prev_ts;
        full_rotations = 0;
        vel_full_rotations = 0;
        velocity = 0.0f;
        lastRaw_ = raw;
        lastRawValid_ = true;
        velocityWindow_.reset(angle_prev_ts, raw);
        return;
      }
      delayMicroseconds(50);
    }

    angle_prev = 0.0f;
    vel_angle_prev = 0.0f;
    angle_prev_ts = micros();
    vel_angle_prev_ts = angle_prev_ts;
    lastRaw_ = 0;
    lastRawValid_ = false;
    velocityWindow_.reset(angle_prev_ts, 0);
  }

  float getSensorAngle() override {
    uint16_t raw = 0;
    if (!readRawChecked(raw)) {
      return -1.0f;
    }

    return rawToAngle(raw);
  }

  void update() override {
    if (prepared_) { prepared_ = false; return; }
    uint16_t raw = 0;
    if (!readRawChecked(raw)) {
      angleOk_ = false;
      return;
    }

    const uint32_t now = micros();
    if (lastRawValid_) {
      const int32_t rawDelta = (int32_t)raw - (int32_t)lastRaw_;
      if (rawDelta > ENCODER_WRAP_THRESHOLD_COUNTS) {
        full_rotations--;
      } else if (rawDelta < -ENCODER_WRAP_THRESHOLD_COUNTS) {
        full_rotations++;
      }
    } else {
      full_rotations = 0;
      velocityWindow_.reset(now, raw);
    }

    lastRaw_ = raw;
    lastRawValid_ = true;
    angle_prev = rawToAngle(raw);
    angle_prev_ts = now;
    angleOk_ = true;
    velocity = velocityWindow_.update(now, extendedRawCount(raw),
      ENCODER_VELOCITY_WINDOW_US, _2PI / float(ENCODER_CPR));
  }

  // SimpleFOC loopFOC() calls update(). Consume the sample already taken before
  // this frame's position controller, avoiding a second SPI transaction.
  void prepareControlSample() {
    update();
    prepared_ = true;
  }

  void setStartupReference(float offset, int direction) {
    full_rotations = control_math::startupTurns(angle_prev, offset, direction);
    vel_full_rotations = full_rotations;
    velocityWindow_.reset(angle_prev_ts, extendedRawCount(lastRaw_));
    velocity = 0.0f;
  }

  float getVelocity() override { return velocity; }

  void resetVelocity() {
    velocityWindow_.reset(angle_prev_ts, extendedRawCount(lastRaw_));
    velocity = 0.0f;
  }

  bool feedbackFresh(uint32_t now) const {
    return lastRawValid_ && !control_math::expired(now, angle_prev_ts, ENCODER_FEEDBACK_TIMEOUT_US);
  }

  void resetTurns() {
    full_rotations = 0;
    vel_full_rotations = 0;
    velocityWindow_.reset(angle_prev_ts, lastRaw_);
    velocity = 0.0f;
  }

  bool angleOk() const {
    return angleOk_;
  }

  EncoderRegisterDiagnostics readRegisterDiagnostics() {
    EncoderRegisterDiagnostics result;
    result.magnitudeOk = readRegisterData(AS5048A_MAGNITUDE_REG, result.magnitude);
    result.diagnosticsOk = readRegisterData(AS5048A_DIAGNOSTICS_REG, result.diagnostics);
    return result;
  }

private:
  static constexpr uint16_t AS5048A_ANGLE_REG = 0x3FFF;
  static constexpr uint16_t AS5048A_CLEAR_ERROR_REG = 0x0001;
  static constexpr uint16_t AS5048A_MAGNITUDE_REG = 0x3FFE;
  static constexpr uint16_t AS5048A_DIAGNOSTICS_REG = 0x3FFD;
  static constexpr uint16_t AS5048A_RESULT_MASK = 0x3FFF;
  static constexpr uint16_t AS5048A_READ_BIT = 0x4000;
  static constexpr uint16_t AS5048A_PARITY_BIT = 0x8000;
  static constexpr uint16_t AS5048A_ERROR_FLAG = 0x4000;
  static constexpr int32_t ENCODER_WRAP_THRESHOLD_COUNTS =
    (int32_t)((float)ENCODER_CPR * 0.8f);

  static bool hasEvenParity(uint16_t value) {
    value ^= value >> 8;
    value ^= value >> 4;
    value ^= value >> 2;
    value ^= value >> 1;
    return (value & 1u) == 0;
  }

  static float rawToAngle(uint16_t raw) {
    return ((float)raw / (float)ENCODER_CPR) * _2PI;
  }

  int64_t extendedRawCount(uint16_t raw) const {
    return (int64_t)full_rotations * (int64_t)ENCODER_CPR + (int64_t)raw;
  }

  static uint16_t makeReadCommand(uint16_t reg) {
    uint16_t command = (reg & AS5048A_RESULT_MASK) | AS5048A_READ_BIT;
    if (!hasEvenParity(command)) {
      command |= AS5048A_PARITY_BIT;
    }
    return command;
  }

  uint16_t transfer16(uint16_t out) {
    uint16_t in = 0;
    sharedSpiUseEncoder();
    gpio_put(csPin_, false);
    busy_wait_at_least_cycles((F_CPU * 350ull + 999999999ull) / 1000000000ull);
    spi_write16_read16_blocking(spi0, &out, &in, 1);
    busy_wait_at_least_cycles((F_CPU * 50ull + 999999999ull) / 1000000000ull);
    gpio_put(csPin_, true);
    busy_wait_at_least_cycles((F_CPU * 350ull + 999999999ull) / 1000000000ull);
    return in;
  }

  void clearErrorFlag() {
    transfer16(makeReadCommand(AS5048A_CLEAR_ERROR_REG));
    delayMicroseconds(2);
    transfer16(makeReadCommand(AS5048A_ANGLE_REG));
    delayMicroseconds(2);
  }

  bool frameToData(uint16_t frame, uint16_t &data) const {
    if (!hasEvenParity(frame)) {
      return false;
    }
    if ((frame & AS5048A_ERROR_FLAG) != 0) {
      return false;
    }
    data = frame & AS5048A_RESULT_MASK;
    return true;
  }

  bool readRegisterData(uint16_t reg, uint16_t &data) {
    transfer16(makeReadCommand(reg));
    const uint16_t frame = transfer16(makeReadCommand(AS5048A_ANGLE_REG));
    return frameToData(frame, data);
  }

  bool readRawChecked(uint16_t &raw) {
    const uint16_t frame = transfer16(makeReadCommand(AS5048A_ANGLE_REG));

    if (!hasEvenParity(frame)) {
      return false;
    }
    if ((frame & AS5048A_ERROR_FLAG) != 0) {
      clearErrorFlag();
      return false;
    }

    const uint16_t candidate = frame & AS5048A_RESULT_MASK;
    raw = candidate;
    return true;
  }

  uint8_t csPin_;
  bool prepared_ = false;
  bool angleOk_ = false;
  uint16_t lastRaw_ = 0;
  bool lastRawValid_ = false;
  control_math::VelocityWindow velocityWindow_;
};
