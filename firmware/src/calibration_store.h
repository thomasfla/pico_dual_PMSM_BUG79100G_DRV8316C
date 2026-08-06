#pragma once

#include <stddef.h>
#include <stdint.h>

struct MotorCalibrationSettings {
  float zeroElectricAngle;
  float sensorOffset;
  int8_t sensorDirectionSign;
  uint8_t reserved[3];
};

struct CalibrationSettings {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint32_t serialNumber;
  MotorCalibrationSettings motor[2];
  uint32_t crc32;
};

void beginCalibrationStore();
CalibrationSettings makeDefaultCalibrationSettings();
bool calibrationSettingsValid(const CalibrationSettings &settings);
bool loadCalibrationSettings(CalibrationSettings &settings);
bool saveCalibrationSettings(CalibrationSettings settings);
