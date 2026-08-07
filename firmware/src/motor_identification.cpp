#include "motor_identification.h"

#include <math.h>

#include "board_config.h"

// SimpleFOC's BLDCMotor::characteriseMotor() applies a 1.5 correction factor
// when estimating phase resistance. Here we report the direct D/Q model value:
// with setPhaseVoltage(0, Ud, angle), the meter-comparable phase resistance is
// R_phase = Ud / Id, and phase-phase resistance is 2 * R_phase.
static constexpr float DQ_STATIC_TEST_CORRECTION = 1.0f;

struct SavedMotorMode {
  TorqueControlType torqueController;
  MotionControlType controller;
  FOCModulationType focModulation;
  float phaseResistance;
  float phaseInductance;
  float kvRating;
  float voltageLimit;
  float currentLimit;
  float target;
  float currentSp;
  DQVoltage_s voltage;
  DQCurrent_s current;
  bool wasEnabled;
};

MotorIdentificationConfig defaultMotorIdentificationConfig() {
  return {
    IDENTIFICATION_TEST_VOLTAGE,
    IDENTIFICATION_SPIN_VOLTAGE,
    IDENTIFICATION_RESISTANCE_RAMP_STEPS,
    IDENTIFICATION_RESISTANCE_RAMP_STEP_US,
    IDENTIFICATION_RESISTANCE_SETTLE_MS,
    IDENTIFICATION_CURRENT_AVERAGE_SAMPLES,
    IDENTIFICATION_CURRENT_AVERAGE_SAMPLE_US,
    IDENTIFICATION_INDUCTANCE_SAMPLES,
    IDENTIFICATION_INDUCTANCE_RISE_US,
    IDENTIFICATION_INDUCTANCE_SETTLE_US,
    IDENTIFICATION_SPINUP_MS,
    IDENTIFICATION_SPIN_MEASURE_MS,
    IDENTIFICATION_SPIN_LOOP_US,
    IDENTIFICATION_MIN_CURRENT_A,
    IDENTIFICATION_MIN_SPEED_RAD_S,
  };
}

static SavedMotorMode saveMotorMode(const BLDCMotor &motor) {
  return {
    motor.torque_controller,
    motor.controller,
    motor.foc_modulation,
    motor.phase_resistance,
    motor.phase_inductance,
    motor.KV_rating,
    motor.voltage_limit,
    motor.current_limit,
    motor.target,
    motor.current_sp,
    motor.voltage,
    motor.current,
    motor.enabled != 0,
  };
}

static void stopMotorVoltage(BLDCMotor &motor) {
  const float angle = motor.sensor != nullptr ? motor.electricalAngle() : 0.0f;
  motor.setPhaseVoltage(0.0f, 0.0f, angle);
  motor.target = 0.0f;
  motor.current_sp = 0.0f;
  motor.voltage.q = 0.0f;
  motor.voltage.d = 0.0f;
}

static void restoreMotorMode(BLDCMotor &motor, const SavedMotorMode &saved) {
  stopMotorVoltage(motor);
  motor.torque_controller = saved.torqueController;
  motor.controller = saved.controller;
  motor.foc_modulation = saved.focModulation;
  motor.phase_resistance = saved.phaseResistance;
  motor.phase_inductance = saved.phaseInductance;
  motor.KV_rating = saved.kvRating;
  motor.voltage_limit = saved.voltageLimit;
  motor.current_limit = saved.currentLimit;
  motor.target = saved.target;
  motor.current_sp = saved.currentSp;
  motor.voltage = saved.voltage;
  motor.current = saved.current;
  if (saved.wasEnabled) {
    motor.enable();
  } else {
    motor.disable();
  }
}

static DQCurrent_s averageDqCurrent(
  CurrentSense &currentSense,
  float electricalAngle,
  uint16_t samples,
  uint16_t sampleUs
) {
  DQCurrent_s sum = {0.0f, 0.0f};
  for (uint16_t i = 0; i < samples; i++) {
    const DQCurrent_s current = currentSense.getFOCCurrents(electricalAngle);
    sum.d += current.d;
    sum.q += current.q;
    if (sampleUs > 0) {
      delayMicroseconds(sampleUs);
    }
  }

  const float invSamples = samples > 0 ? 1.0f / (float)samples : 0.0f;
  return {
    sum.d * invSamples,
    sum.q * invSamples,
  };
}

static void rampStaticDVoltage(
  BLDCMotor &motor,
  float electricalAngle,
  float voltage,
  uint16_t steps,
  uint16_t stepUs
) {
  if (steps == 0) {
    motor.setPhaseVoltage(0.0f, voltage, electricalAngle);
    return;
  }

  for (uint16_t i = 1; i <= steps; i++) {
    const float u = voltage * (float)i / (float)steps;
    motor.setPhaseVoltage(0.0f, u, electricalAngle);
    delayMicroseconds(stepUs);
  }
}

static bool measureResistance(
  BLDCMotor &motor,
  CurrentSense &currentSense,
  const MotorIdentificationConfig &config,
  MotorIdentificationResult &result
) {
  const float electricalAngle = motor.electricalAngle();
  stopMotorVoltage(motor);
  delay(100);

  const DQCurrent_s zero = averageDqCurrent(
    currentSense,
    electricalAngle,
    config.currentAverageSamples,
    config.currentAverageSampleUs
  );

  rampStaticDVoltage(
    motor,
    electricalAngle,
    config.testVoltage,
    config.resistanceRampSteps,
    config.resistanceRampStepUs
  );
  delay(config.resistanceSettleMs);

  const DQCurrent_s loaded = averageDqCurrent(
    currentSense,
    electricalAngle,
    config.currentAverageSamples,
    config.currentAverageSampleUs
  );

  stopMotorVoltage(motor);
  const float idDelta = fabsf(loaded.d - zero.d);
  if (idDelta < config.minCurrentA) {
    return false;
  }

  result.phaseResistanceOhm = config.testVoltage / (DQ_STATIC_TEST_CORRECTION * idDelta);
  result.phaseToPhaseResistanceOhm = 2.0f * result.phaseResistanceOhm;
  result.resistanceOk =
    isfinite(result.phaseResistanceOhm) && result.phaseResistanceOhm > 0.0f;
  return result.resistanceOk;
}

static bool measureAxisInductance(
  BLDCMotor &motor,
  CurrentSense &currentSense,
  float axisElectricalAngle,
  float phaseResistanceOhm,
  const MotorIdentificationConfig &config,
  float &inductanceH
) {
  float sum = 0.0f;
  uint16_t accepted = 0;

  stopMotorVoltage(motor);
  delay(20);

  for (uint16_t i = 0; i < config.inductanceSamples; i++) {
    const DQCurrent_s zero = averageDqCurrent(currentSense, axisElectricalAngle, 8, 20);

    motor.setPhaseVoltage(0.0f, config.testVoltage, axisElectricalAngle);
    const uint32_t t0 = micros();
    delayMicroseconds(config.inductanceRiseUs);
    const DQCurrent_s stepped = currentSense.getFOCCurrents(axisElectricalAngle);
    const uint32_t t1 = micros();
    stopMotorVoltage(motor);

    const float dt = (float)(t1 - t0) * 1.0e-6f;
    const float idDelta = stepped.d - zero.d;
    const float remainingVoltage = config.testVoltage - phaseResistanceOhm * idDelta;
    if (dt > 0.0f &&
        idDelta > 0.0f &&
        remainingVoltage > 0.0f &&
        remainingVoltage < config.testVoltage) {
      const float ratio = remainingVoltage / config.testVoltage;
      const float l = fabsf(
        -(phaseResistanceOhm * dt) / logf(ratio)
      ) / DQ_STATIC_TEST_CORRECTION;
      if (isfinite(l) && l > 0.0f) {
        sum += l;
        accepted++;
      }
    }

    delayMicroseconds(config.inductanceSettleUs);
  }

  if (accepted == 0) {
    inductanceH = 0.0f;
    return false;
  }

  inductanceH = sum / (float)accepted;
  return true;
}

static bool measureInductance(
  BLDCMotor &motor,
  CurrentSense &currentSense,
  const MotorIdentificationConfig &config,
  MotorIdentificationResult &result
) {
  if (!result.resistanceOk) {
    return false;
  }

  const float electricalAngle = motor.electricalAngle();
  const bool ldOk = measureAxisInductance(
    motor,
    currentSense,
    electricalAngle,
    result.phaseResistanceOhm,
    config,
    result.ldH
  );
  const bool lqOk = measureAxisInductance(
    motor,
    currentSense,
    _normalizeAngle(electricalAngle + _PI_2),
    result.phaseResistanceOhm,
    config,
    result.lqH
  );

  result.inductanceOk = ldOk && lqOk;
  return result.inductanceOk;
}

static bool runSpinDirection(
  BLDCMotor &motor,
  CurrentSense &currentSense,
  float voltage,
  const MotorIdentificationConfig &config,
  float &velocityRadS,
  float &currentA
) {
  uint32_t startMs = millis();
  while ((millis() - startMs) < config.spinupMs) {
    motor.loopFOC();
    motor.move(voltage);
    delayMicroseconds(config.spinLoopUs);
  }

  float velocitySum = 0.0f;
  float currentSum = 0.0f;
  uint32_t samples = 0;
  startMs = millis();
  while ((millis() - startMs) < config.spinMeasureMs) {
    motor.loopFOC();
    motor.move(voltage);
    const DQCurrent_s current = currentSense.getFOCCurrents(motor.electrical_angle);
    velocitySum += motor.shaft_velocity;
    currentSum += current.q;
    samples++;
    delayMicroseconds(config.spinLoopUs);
  }

  stopMotorVoltage(motor);
  delay(150);

  if (samples == 0) {
    velocityRadS = 0.0f;
    currentA = 0.0f;
    return false;
  }

  velocityRadS = velocitySum / (float)samples;
  currentA = currentSum / (float)samples;
  return fabsf(velocityRadS) >= config.minSpeedRadS;
}

static bool measureBemf(
  BLDCMotor &motor,
  CurrentSense &currentSense,
  const MotorIdentificationConfig &config,
  MotorIdentificationResult &result
) {
  if (!result.resistanceOk) {
    return false;
  }

  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;
  motor.phase_resistance = NOT_SET;
  motor.phase_inductance = NOT_SET;
  motor.KV_rating = NOT_SET;
  motor.voltage_limit = _constrain(config.spinVoltage, 0.0f, motor.driver->voltage_limit);
  motor.current_limit = GM3506_PEAK_CURRENT_A;
  motor.enable();

  float velocityPos = 0.0f;
  float currentPos = 0.0f;
  const bool posOk = runSpinDirection(
    motor,
    currentSense,
    motor.voltage_limit,
    config,
    velocityPos,
    currentPos
  );

  float velocityNeg = 0.0f;
  float currentNeg = 0.0f;
  const bool negOk = runSpinDirection(
    motor,
    currentSense,
    -motor.voltage_limit,
    config,
    velocityNeg,
    currentNeg
  );

  float keSum = 0.0f;
  uint8_t accepted = 0;
  if (posOk) {
    const float bemfVoltage = motor.voltage_limit - result.phaseResistanceOhm * fabsf(currentPos);
    if (bemfVoltage > 0.0f) {
      keSum += bemfVoltage / fabsf(velocityPos);
      accepted++;
    }
  }
  if (negOk) {
    const float bemfVoltage = motor.voltage_limit - result.phaseResistanceOhm * fabsf(currentNeg);
    if (bemfVoltage > 0.0f) {
      keSum += bemfVoltage / fabsf(velocityNeg);
      accepted++;
    }
  }

  if (accepted == 0) {
    return false;
  }

  result.bemfConstantVPerRadS = keSum / (float)accepted;
  result.kvRatingRpmPerVolt =
    1.0f / (result.bemfConstantVPerRadS * _SQRT3 * _RPM_TO_RADS);
  result.noLoadVelocityRadS =
    (fabsf(velocityPos) + fabsf(velocityNeg)) /
    (float)((posOk ? 1 : 0) + (negOk ? 1 : 0));
  result.noLoadVelocityRpm = result.noLoadVelocityRadS / _RPM_TO_RADS;
  result.noLoadCurrentA =
    (fabsf(currentPos) + fabsf(currentNeg)) /
    (float)((posOk ? 1 : 0) + (negOk ? 1 : 0));
  result.bemfOk =
    isfinite(result.bemfConstantVPerRadS) &&
    isfinite(result.kvRatingRpmPerVolt) &&
    result.bemfConstantVPerRadS > 0.0f;
  return result.bemfOk;
}

static void printOk(Print &out, bool ok) {
  out.print(ok ? "ok" : "failed");
}

static void printResult(const char *label, Print &out, const MotorIdentificationResult &result) {
  out.print(label);
  out.println(" identification result:");
  out.print("  R phase=");
  if (result.resistanceOk) {
    out.print(result.phaseResistanceOhm, 5);
    out.print(" ohm, phase-phase=");
    out.print(result.phaseToPhaseResistanceOhm, 5);
    out.println(" ohm");
  } else {
    out.println("failed");
  }

  out.print("  Ld=");
  if (result.inductanceOk) {
    out.print(result.ldH * 1000.0f, 5);
    out.print(" mH, Lq=");
    out.print(result.lqH * 1000.0f, 5);
    out.println(" mH");
  } else {
    out.println("failed");
  }

  out.print("  BEMF Ke=");
  if (result.bemfOk) {
    out.print(result.bemfConstantVPerRadS, 6);
    out.print(" V/(rad/s), SimpleFOC KV=");
    out.print(result.kvRatingRpmPerVolt, 2);
    out.println(" rpm/V");
    out.print("  no-load speed=");
    out.print(result.noLoadVelocityRadS, 3);
    out.print(" rad/s (");
    out.print(result.noLoadVelocityRpm, 1);
    out.print(" rpm), no-load Iq=");
    out.print(result.noLoadCurrentA, 4);
    out.println(" A");
  } else {
    out.println("failed");
  }
}

bool identifyMotorParameters(
  const char *label,
  BLDCMotor &motor,
  CurrentSense &currentSense,
  Print &out,
  const MotorIdentificationConfig &config,
  MotorIdentificationResult &result
) {
  result = {};

  if (motor.driver == nullptr ||
      !motor.driver->initialized ||
      !currentSense.initialized ||
      motor.sensor == nullptr) {
    out.print(label);
    out.println(": identification skipped, motor/current/sensor not ready.");
    return false;
  }

  const SavedMotorMode saved = saveMotorMode(motor);
  motor.enable();

  out.print(label);
  out.print(": R/L static test at ");
  out.print(config.testVoltage, 3);
  out.println(" V. Keep the rotor still.");
  const bool rOk = measureResistance(motor, currentSense, config, result);
  out.print(label);
  out.print(": resistance ");
  printOk(out, rOk);
  out.println();

  const bool lOk = measureInductance(motor, currentSense, config, result);
  out.print(label);
  out.print(": inductance ");
  printOk(out, lOk);
  out.println();

  out.print(label);
  out.print(": back-EMF spin test at +/-");
  out.print(config.spinVoltage, 3);
  out.println(" V. Motor must be free to spin.");
  const bool bemfOk = measureBemf(motor, currentSense, config, result);
  out.print(label);
  out.print(": back-EMF ");
  printOk(out, bemfOk);
  out.println();

  restoreMotorMode(motor, saved);
  printResult(label, out, result);
  return result.resistanceOk || result.inductanceOk || result.bemfOk;
}
