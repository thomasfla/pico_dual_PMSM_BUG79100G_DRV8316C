#include "calibration_store.h"

#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>
#include <stddef.h>
#include "board_config.h"

static constexpr uint32_t CALIBRATION_MAGIC = 0x314C4143;  // 'CAL1'
static constexpr uint16_t CALIBRATION_VERSION = 1;

static uint32_t fnv1a32(const uint8_t *data, size_t length) {
  uint32_t hash = 2166136261u;
  for (size_t i = 0; i < length; i++) {
    hash ^= data[i];
    hash *= 16777619u;
  }
  return hash;
}

static uint32_t calibrationCrc(const CalibrationSettings &settings) {
  return fnv1a32(
    (const uint8_t *)&settings,
    offsetof(CalibrationSettings, crc32)
  );
}

static bool validDirectionSign(int8_t sign) {
  return sign >= -1 && sign <= 1;
}

static bool validAngleOrNotSet(float value) {
  return value == NOT_SET || (isfinite(value) && value > -100.0f && value < 100.0f);
}

static bool validSensorOffset(float value) {
  return isfinite(value) && value > -1000.0f && value < 1000.0f;
}

void beginCalibrationStore() {
  EEPROM.begin(CALIBRATION_EEPROM_BYTES);
}

CalibrationSettings makeDefaultCalibrationSettings() {
  CalibrationSettings settings = {};
  settings.magic = CALIBRATION_MAGIC;
  settings.version = CALIBRATION_VERSION;
  settings.size = sizeof(CalibrationSettings);
  settings.serialNumber = 0;
  settings.motor[0].zeroElectricAngle = M0_ZERO_ELECTRIC_ANGLE;
  settings.motor[0].sensorOffset = M0_SENSOR_OFFSET;
  settings.motor[0].sensorDirectionSign = M0_SENSOR_DIRECTION_SIGN;
  settings.motor[1].zeroElectricAngle = M1_ZERO_ELECTRIC_ANGLE;
  settings.motor[1].sensorOffset = M1_SENSOR_OFFSET;
  settings.motor[1].sensorDirectionSign = M1_SENSOR_DIRECTION_SIGN;
  settings.crc32 = calibrationCrc(settings);
  return settings;
}

bool calibrationSettingsValid(const CalibrationSettings &settings) {
  if (settings.magic != CALIBRATION_MAGIC ||
      settings.version != CALIBRATION_VERSION ||
      settings.size != sizeof(CalibrationSettings)) {
    return false;
  }
  for (uint8_t i = 0; i < 2; i++) {
    if (!validDirectionSign(settings.motor[i].sensorDirectionSign) ||
        !validAngleOrNotSet(settings.motor[i].zeroElectricAngle) ||
        !validSensorOffset(settings.motor[i].sensorOffset)) {
      return false;
    }
  }
  return calibrationCrc(settings) == settings.crc32;
}

bool loadCalibrationSettings(CalibrationSettings &settings) {
  EEPROM.get(0, settings);
  return calibrationSettingsValid(settings);
}

bool saveCalibrationSettings(CalibrationSettings settings) {
  settings.magic = CALIBRATION_MAGIC;
  settings.version = CALIBRATION_VERSION;
  settings.size = sizeof(CalibrationSettings);
  settings.crc32 = calibrationCrc(settings);
  EEPROM.put(0, settings);
  return EEPROM.commit();
}
