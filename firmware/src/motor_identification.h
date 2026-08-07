#pragma once

#include <Arduino.h>
#include <SimpleFOC.h>

struct MotorIdentificationConfig {
  float testVoltage;
  float spinVoltage;
  uint16_t resistanceRampSteps;
  uint16_t resistanceRampStepUs;
  uint16_t resistanceSettleMs;
  uint16_t currentAverageSamples;
  uint16_t currentAverageSampleUs;
  uint16_t inductanceSamples;
  uint16_t inductanceRiseUs;
  uint16_t inductanceSettleUs;
  uint16_t spinupMs;
  uint16_t spinMeasureMs;
  uint16_t spinLoopUs;
  float minCurrentA;
  float minSpeedRadS;
};

struct MotorIdentificationResult {
  bool resistanceOk = false;
  bool inductanceOk = false;
  bool bemfOk = false;
  float phaseResistanceOhm = 0.0f;
  float phaseToPhaseResistanceOhm = 0.0f;
  float ldH = 0.0f;
  float lqH = 0.0f;
  float bemfConstantVPerRadS = 0.0f;
  float kvRatingRpmPerVolt = 0.0f;
  float noLoadCurrentA = 0.0f;
  float noLoadVelocityRadS = 0.0f;
  float noLoadVelocityRpm = 0.0f;
};

MotorIdentificationConfig defaultMotorIdentificationConfig();

bool identifyMotorParameters(
  const char *label,
  BLDCMotor &motor,
  CurrentSense &currentSense,
  Print &out,
  const MotorIdentificationConfig &config,
  MotorIdentificationResult &result
);
