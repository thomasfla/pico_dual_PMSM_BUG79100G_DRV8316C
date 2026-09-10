#include "firmware.h"
#include "motor_identification.h"
#include "control_math.h"
#include "emergency_stop.h"
#include <cstdlib>

void clearSerialInput() {
  delay(20);
  while (Serial.available() > 0) {
    Serial.read();
  }
}

size_t readSerialLine(char *buffer, size_t length) {
  size_t count = 0;
  if (length == 0) {
    return 0;
  }

  while (true) {
    serviceEmergencyStop();
    while (Serial.available() > 0) {
      const int value = Serial.read();
      if (value < 0) {
        continue;
      }

      const char c = (char)value;
      if (c == '\r' || c == '\n') {
        buffer[count] = '\0';
        Serial.println();

        const char pairedNewline = (c == '\r') ? '\n' : '\r';
        const uint32_t pairStartMs = millis();
        while ((millis() - pairStartMs) < 5) {
          if (Serial.available() <= 0) {
            statusLed.serviceStartup();
            delay(1);
            continue;
          }
          if (Serial.peek() == pairedNewline) {
            Serial.read();
          }
          break;
        }

        return count;
      }

      if (c == '\b' || c == 0x7F) {
        if (count > 0) {
          count--;
          Serial.print("\b \b");
        }
        continue;
      }

      if (count + 1 < length) {
        buffer[count++] = c;
        if (c >= ' ' && c <= '~') {
          Serial.print(c);
        }
      }
    }
    statusLed.serviceStartup();
    delay(1);
  }
}

char firstCommandChar(const char *line) {
  while (*line == ' ' || *line == '\t') {
    line++;
  }
  return *line;
}

bool serialConfirm(const char *prompt, bool defaultYes) {
  char line[16];
  while (true) {
    Serial.print(prompt);
    Serial.print(defaultYes ? " [Y/n] " : " [y/N] ");
    readSerialLine(line, sizeof(line));

    const char c = firstCommandChar(line);
    if (c == '\0') {
      return defaultYes;
    }
    if (c == 'y' || c == 'Y') {
      return true;
    }
    if (c == 'n' || c == 'N') {
      return false;
    }
    Serial.println("Please answer y or n.");
  }
}

uint32_t readUint32WithDefault(const char *prompt, uint32_t currentValue) {
  char line[24];
  while (true) {
    Serial.print(prompt);
    Serial.print(" (Enter keeps ");
    Serial.print(currentValue);
    Serial.print("): ");
    readSerialLine(line, sizeof(line));

    const char *start = line;
    while (*start == ' ' || *start == '\t') {
      start++;
    }
    if (*start == '\0') {
      return currentValue;
    }

    char *end = nullptr;
    const unsigned long parsed = strtoul(start, &end, 10);
    while (*end == ' ' || *end == '\t') {
      end++;
    }
    if (*end == '\0') {
      return (uint32_t)parsed;
    }
    Serial.println("Please enter a decimal number or an empty line.");
  }
}

static void waitForEnter(const char *prompt) {
  char line[8];
  Serial.println(prompt);
  readSerialLine(line, sizeof(line));
}

static void printAngleSetting(float value) {
  if (value == NOT_SET) {
    Serial.print("NOT_SET");
  } else {
    Serial.print(value, 6);
  }
}

static void printMotorCalibration(const char *label, const MotorCalibrationSettings &calibration) {
  Serial.print(label);
  Serial.print(": dir=");
  Serial.print(calibration.sensorDirectionSign);
  Serial.print(" zero_elec=");
  printAngleSetting(calibration.zeroElectricAngle);
  Serial.print(" sensor_offset=");
  Serial.println(calibration.sensorOffset, 6);
}

static void printCalibrationSettings(const CalibrationSettings &settings) {
  Serial.print("serial=");
  Serial.println(settings.serialNumber);
  printMotorCalibration("M0", settings.motor[0]);
  printMotorCalibration("M1", settings.motor[1]);
}

SerialBootMode serialBootModeRequested() {
  const uint32_t startMs = millis();
  while ((millis() - startMs) < CALIBRATION_ENTRY_WAIT_MS) {
    if (Serial.available() > 0) {
      const int value = Serial.read();
      if (value == '\r' || value == '\n') {
        continue;
      }
      if (value == '!') {
        return SerialBootMode::Calibration;
      }
      if (value == '?') {
        return SerialBootMode::BoardTest;
      }
      if (value >= 0) {
        usbPendingByte = (uint8_t)value;
        usbPendingByteValid = true;
      }
      return SerialBootMode::Control;
    }
    statusLed.serviceStartup();
    delay(1);
  }
  return SerialBootMode::Control;
}

static bool prepareMotorForMechanicalCalibration(
  BLDCMotor &motor,
  DRV8316Driver3PWM &driver,
  CurrentSense &currentSense,
  Sensor &sensor,
  PositionHoldConfig &config,
  bool encoderAllowed,
  bool currentFeedbackOk
) {
  if (motor.sensor != nullptr && motor.driver != nullptr) {
    return true;
  }
  if (!encoderAllowed || !currentFeedbackOk || config.sensorDirectionSign == 0) {
    return false;
  }
  if (!configureMotor(motor, driver, currentSense, sensor, config)) {
    return false;
  }
  motor.disable();
  return true;
}

static bool runElectricalCalibrationForMotor(
  const char *label,
  uint8_t motorIndex,
  BLDCMotor &motor,
  DRV8316Driver3PWM &driver,
  CurrentSense &currentSense,
  CheckedAS5048ASensor &encoder,
  PositionHoldConfig &config,
  bool encoderAllowed,
  bool currentFeedbackOk,
  CalibrationSettings &settings
) {
  if (!encoderAllowed) {
    Serial.print(label);
    Serial.println(": encoder not healthy, skipped.");
    driver.disable();
    return false;
  }
  if (!currentFeedbackOk) {
    Serial.print(label);
    Serial.println(": current feedback not ready, skipped.");
    driver.disable();
    return false;
  }

  PositionHoldConfig calibrationConfig = config;
  calibrationConfig.sensorDirectionSign = 0;
  calibrationConfig.zeroElectricAngle = NOT_SET;
  calibrationConfig.sensorOffset = 0.0f;

  Serial.print(label);
  Serial.println(": running SimpleFOC electrical alignment.");
  if (!configureMotor(motor, driver, currentSense, encoder, calibrationConfig)) {
    Serial.print(label);
    Serial.println(": motor init failed.");
    driver.disable();
    return false;
  }

  const bool focOk = motor.initFOC() != 0 && !emergency_stop::active();
  setIqTarget(motor, 0.0f);
  motor.disable();
  if (!focOk) {
    serviceEmergencyStop();
    Serial.print(label);
    Serial.println(": electrical alignment failed.");
    return false;
  }

  const int8_t directionSign = signFromDirection(motor.sensor_direction);
  if (directionSign == 0 || motor.zero_electric_angle == NOT_SET) {
    Serial.print(label);
    Serial.println(": invalid alignment result.");
    return false;
  }

  settings.motor[motorIndex].sensorDirectionSign = directionSign;
  settings.motor[motorIndex].zeroElectricAngle = motor.zero_electric_angle;
  settings.motor[motorIndex].sensorOffset = 0.0f;
  applyMotorCalibration(config, settings.motor[motorIndex]);

  Serial.print(label);
  Serial.print(": electrical calibration ok, dir=");
  Serial.print(directionSign);
  Serial.print(" zero_elec=");
  Serial.println(motor.zero_electric_angle, 6);
  return true;
}

static bool captureMechanicalZeroForMotor(
  const char *label,
  uint8_t motorIndex,
  BLDCMotor &motor,
  PositionHoldConfig &config,
  CalibrationSettings &settings
) {
  if (motor.sensor == nullptr || settings.motor[motorIndex].sensorDirectionSign == 0) {
    Serial.print(label);
    Serial.println(": not configured, mechanical zero skipped.");
    return false;
  }

  motor.sensor_direction = directionFromSign(settings.motor[motorIndex].sensorDirectionSign);
  motor.sensor_offset = 0.0f;
  motor.sensor->update();
  delay(5);
  motor.sensor->update();

  const float sensorAngle = motor.sensor->getMechanicalAngle();
  auto &encoder = static_cast<CheckedAS5048ASensor &>(*motor.sensor);
  if (!encoder.angleOk() || !encoder.feedbackFresh(micros()) || !isfinite(sensorAngle)) {
    Serial.print(label);
    Serial.println(": sensor read failed, mechanical zero skipped.");
    return false;
  }

  const float sensorOffset = (float)motor.sensor_direction * sensorAngle;
  settings.motor[motorIndex].sensorOffset = sensorOffset;
  config.sensorOffset = sensorOffset;
  motor.sensor_offset = sensorOffset;
  encoder.resetTurns();
  motor.shaft_angle = motor.shaftAngle();

  Serial.print(label);
  Serial.print(": mechanical zero captured, sensor_offset=");
  Serial.println(sensorOffset, 6);
  return true;
}

static bool prepareMotorForIdentification(
  const char *label,
  BLDCMotor &motor,
  DRV8316Driver3PWM &driver,
  CurrentSense &currentSense,
  CheckedAS5048ASensor &encoder,
  PositionHoldConfig &config,
  bool encoderAllowed,
  bool currentFeedbackOk
) {
  if (!encoderAllowed) {
    Serial.print(label);
    Serial.println(": encoder not healthy, identification skipped.");
    driver.disable();
    return false;
  }
  if (!currentFeedbackOk) {
    Serial.print(label);
    Serial.println(": current feedback not ready, identification skipped.");
    driver.disable();
    return false;
  }
  if (config.sensorDirectionSign == 0 || config.zeroElectricAngle == NOT_SET) {
    Serial.print(label);
    Serial.println(": run electrical calibration before identification.");
    driver.disable();
    return false;
  }
  if (!configureMotor(motor, driver, currentSense, encoder, config)) {
    Serial.print(label);
    Serial.println(": motor init failed, identification skipped.");
    driver.disable();
    return false;
  }
  if (motor.initFOC() == 0) {
    Serial.print(label);
    Serial.println(": FOC init failed, identification skipped.");
    motor.disable();
    driver.disable();
    return false;
  }

  setIqTarget(motor, 0.0f);
  return true;
}

void runCalibrationWizard() {
  clearSerialInput();

  Serial.println();
  Serial.println("Dual PMSM calibration wizard");
  Serial.println("Binary protocol is disabled in this boot mode.");
  Serial.println("Keep motors unloaded and able to move during electrical calibration.");
  Serial.println();
  Serial.println("Current stored calibration:");
  printCalibrationSettings(activeCalibrationSettings);
  Serial.println();

  CalibrationSettings workingSettings = activeCalibrationSettings;
  PositionHoldConfig workingConfig0 = runtimeConfig0;
  PositionHoldConfig workingConfig1 = runtimeConfig1;

  workingSettings.serialNumber =
    readUint32WithDefault("Unit serial number", workingSettings.serialNumber);
  const bool runElectrical = serialConfirm("Run electrical angle calibration?", true);
  const bool runMechanical = serialConfirm("Run mechanical zero calibration?", true);
  const bool runIdentification = serialConfirm("Run motor parameter identification?", false);

  if (runElectrical || runMechanical || runIdentification) {
    Serial.println("Initializing motor hardware...");
    const MotorHardwareStatus hardware = initializeMotorHardware();
    Serial.print("VBUS=");
    Serial.print(hardware.busVoltage, 3);
    Serial.println(" V");

    if (runElectrical) {
      waitForEnter("Press Enter to calibrate M0 electrical angle.");
      runElectricalCalibrationForMotor(
        "M0",
        0,
        motor0,
        driver0,
        currentSense0,
        encoder0,
        workingConfig0,
        hardware.encoder0Allowed,
        hardware.currentFeedback0Ok,
        workingSettings
      );

      waitForEnter("Press Enter to calibrate M1 electrical angle.");
      runElectricalCalibrationForMotor(
        "M1",
        1,
        motor1,
        driver1,
        currentSense1,
        encoder1,
        workingConfig1,
        hardware.encoder1Allowed,
        hardware.currentFeedback1Ok,
        workingSettings
      );
    } else {
      prepareMotorForMechanicalCalibration(
        motor0,
        driver0,
        currentSense0,
        encoder0,
        workingConfig0,
        hardware.encoder0Allowed,
        hardware.currentFeedback0Ok
      );
      prepareMotorForMechanicalCalibration(
        motor1,
        driver1,
        currentSense1,
        encoder1,
        workingConfig1,
        hardware.encoder1Allowed,
        hardware.currentFeedback1Ok
      );
    }

    if (runMechanical) {
      waitForEnter("Move M0 to mechanical zero, then press Enter.");
      captureMechanicalZeroForMotor("M0", 0, motor0, workingConfig0, workingSettings);

      waitForEnter("Move M1 to mechanical zero, then press Enter.");
      captureMechanicalZeroForMotor("M1", 1, motor1, workingConfig1, workingSettings);
    }

    if (runIdentification) {
      MotorIdentificationConfig identificationConfig = defaultMotorIdentificationConfig();

      Serial.println();
      Serial.println("Motor identification prints estimates only; it does not save motor parameters.");
      Serial.println("Keep the motor unloaded. It will hold still for R/L, then spin for BEMF.");

      waitForEnter("Press Enter to identify M0.");
      if (prepareMotorForIdentification(
            "M0",
            motor0,
            driver0,
            currentSense0,
            encoder0,
            workingConfig0,
            hardware.encoder0Allowed,
            hardware.currentFeedback0Ok
          )) {
        MotorIdentificationResult result;
        identifyMotorParameters(
          "M0",
          motor0,
          currentSense0,
          Serial,
          identificationConfig,
          result
        );
      }

      waitForEnter("Press Enter to identify M1.");
      if (prepareMotorForIdentification(
            "M1",
            motor1,
            driver1,
            currentSense1,
            encoder1,
            workingConfig1,
            hardware.encoder1Allowed,
            hardware.currentFeedback1Ok
          )) {
        MotorIdentificationResult result;
        identifyMotorParameters(
          "M1",
          motor1,
          currentSense1,
          Serial,
          identificationConfig,
          result
        );
      }
    }

    setIqTarget(motor0, 0.0f);
    setIqTarget(motor1, 0.0f);
    motor0.disable();
    motor1.disable();
    driver0.disable();
    driver1.disable();
  }

  Serial.println();
  Serial.println("Candidate calibration:");
  printCalibrationSettings(workingSettings);
  Serial.println();

  if (serialConfirm("Save candidate calibration to flash?", false)) {
    if (saveCalibrationSettings(workingSettings)) {
      activeCalibrationSettings = workingSettings;
      applyCalibrationSettings(activeCalibrationSettings);
      Serial.println("Calibration saved.");
    } else {
      Serial.println("Calibration save failed.");
    }
  } else {
    Serial.println("Calibration not saved.");
  }

  Serial.println("Reset the board to start the binary control protocol.");
}
