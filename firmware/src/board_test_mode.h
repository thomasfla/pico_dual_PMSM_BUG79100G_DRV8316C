#pragma once

#include <float.h>

static constexpr float BOARD_TEST_OPENLOOP_DEFAULT_VOLTAGE = 1.0f;
static constexpr float BOARD_TEST_OPENLOOP_MAX_VOLTAGE = 3.0f;
static constexpr float BOARD_TEST_OPENLOOP_DEFAULT_VELOCITY = 4.0f;
static constexpr uint32_t BOARD_TEST_OPENLOOP_DEFAULT_MS = 3000;
static constexpr uint16_t BOARD_TEST_OPENLOOP_LOOP_US = 500;
static constexpr uint16_t BOARD_TEST_CURRENT_SAMPLES = 512;
static constexpr uint16_t BOARD_TEST_CURRENT_SAMPLE_US = 50;
static constexpr float BOARD_TEST_CURRENT_MEAN_LIMIT_A = 0.35f;
static constexpr float BOARD_TEST_CURRENT_STD_LIMIT_A = 0.35f;
static constexpr float BOARD_TEST_CURRENT_PKPK_LIMIT_A = 2.0f;

struct BoardTestStats {
  uint32_t samples = 0;
  float mean = 0.0f;
  float m2 = 0.0f;
  float minValue = FLT_MAX;
  float maxValue = -FLT_MAX;

  void add(float value) {
    samples++;
    const float delta = value - mean;
    mean += delta / (float)samples;
    const float delta2 = value - mean;
    m2 += delta * delta2;
    if (value < minValue) {
      minValue = value;
    }
    if (value > maxValue) {
      maxValue = value;
    }
  }

  float stddev() const {
    return samples > 1 ? sqrtf(m2 / (float)(samples - 1)) : 0.0f;
  }

  float pkpk() const {
    return samples > 0 ? maxValue - minValue : 0.0f;
  }
};

struct BoardTestCurrentStats {
  BoardTestStats a;
  BoardTestStats b;
  BoardTestStats c;
};

static float readFloatWithDefault(
  const char *prompt,
  float currentValue,
  float minValue,
  float maxValue
) {
  char line[32];
  while (true) {
    Serial.print(prompt);
    Serial.print(" (Enter keeps ");
    Serial.print(currentValue, 3);
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
    const float parsed = strtof(start, &end);
    while (*end == ' ' || *end == '\t') {
      end++;
    }
    if (*end == '\0' && isfinite(parsed) && parsed >= minValue && parsed <= maxValue) {
      return parsed;
    }

    Serial.print("Please enter a number from ");
    Serial.print(minValue, 3);
    Serial.print(" to ");
    Serial.print(maxValue, 3);
    Serial.println(".");
  }
}

static void printBoardTestResult(bool ok) {
  Serial.println(ok ? "PASS" : "FAIL");
}

static bool drvStatusHasSpiFault(const DRV8316Status &status) {
  return status.status.SPI_FLT ||
    status.status2.SPI_ADDR_FLT ||
    status.status2.SPI_SCLK_FLT ||
    status.status2.SPI_PARITY;
}

static bool boardTestDrvCommunication(
  const char *label,
  uint8_t csPin,
  DRV8316Driver3PWM &driver
) {
  DRV8316Status status = driver.getStatus();
  const DRV8316_PWMMode pwmMode = driver.getPWMMode();
  const DRV8316_CSAGain csaGain = driver.getCurrentSenseGain();
  const bool spiOk = !drvStatusHasSpiFault(status);
  const bool readbackOk =
    pwmMode == DRV8316_PWMMode::PWM3_Mode &&
    csaGain == DRV8316_CSAGain::Gain_0V1875;
  const bool ok = driver.initialized && spiOk && readbackOk;

  Serial.print(label);
  Serial.print(" DRV CS=");
  Serial.print(csPin);
  Serial.print(" initialized=");
  Serial.print(driver.initialized ? 1 : 0);
  Serial.print(" pwm_mode=");
  Serial.print((uint8_t)pwmMode);
  Serial.print(" csa_gain_raw=");
  Serial.print((uint8_t)csaGain);
  Serial.print(" nFAULT=");
  Serial.print(digitalRead(GPIO_DRV_Mx_nFAULT) == LOW ? "LOW" : "HIGH");
  Serial.print(" status=0x");
  Serial.print(status.status.reg, HEX);
  Serial.print(" status1=0x");
  Serial.print(status.status1.reg, HEX);
  Serial.print(" status2=0x");
  Serial.print(status.status2.reg, HEX);
  Serial.print(" ");
  printBoardTestResult(ok);

  if (!readbackOk) {
    Serial.println("  Expected DRV readback: pwm_mode=2 (3PWM), csa_gain_raw=1.");
  }
  if (!spiOk) {
    Serial.println("  DRV reports an SPI fault bit.");
  }
  return ok;
}

static bool boardTestEncoderCommunication(
  const char *label,
  CheckedAS5048ASensor &encoder
) {
  const float angle = encoder.getSensorAngle();
  const bool angleOk = angle >= 0.0f && isfinite(angle);
  const EncoderRegisterDiagnostics regs = encoder.readRegisterDiagnostics();
  const bool diagnosticsOk = encoderDiagnosticsHealthy(regs);
  const bool ok = angleOk && diagnosticsOk;

  Serial.print(label);
  Serial.print(" encoder angle=");
  if (angleOk) {
    Serial.print(angle, 6);
  } else {
    Serial.print("ERR");
  }
  Serial.print(" mag=");
  if (regs.magnitudeOk) {
    Serial.print(regs.magnitude);
  } else {
    Serial.print("ERR");
  }
  if (regs.diagnosticsOk) {
    Serial.print(" diag=0x");
    Serial.print(regs.diagnostics, HEX);
  } else {
    Serial.print(" diag=");
    Serial.print("ERR");
  }
  Serial.print(" ocf=");
  Serial.print(regs.diagnosticsOk ? encoderOcf(regs.diagnostics) : 0);
  Serial.print(" cof=");
  Serial.print(regs.diagnosticsOk ? encoderCof(regs.diagnostics) : 0);
  Serial.print(" low=");
  Serial.print(regs.diagnosticsOk ? encoderCompLow(regs.diagnostics) : 0);
  Serial.print(" high=");
  Serial.print(regs.diagnosticsOk ? encoderCompHigh(regs.diagnostics) : 0);
  Serial.print(" ");
  printBoardTestResult(ok);
  return ok;
}

static bool boardTestCurrentPhaseOk(const BoardTestStats &stats) {
  return fabsf(stats.mean) <= BOARD_TEST_CURRENT_MEAN_LIMIT_A &&
    stats.stddev() <= BOARD_TEST_CURRENT_STD_LIMIT_A &&
    stats.pkpk() <= BOARD_TEST_CURRENT_PKPK_LIMIT_A;
}

static bool boardTestCurrentStatsOk(const BoardTestCurrentStats &stats) {
  return boardTestCurrentPhaseOk(stats.a) &&
    boardTestCurrentPhaseOk(stats.b) &&
    boardTestCurrentPhaseOk(stats.c);
}

static void boardTestPrintCurrentStats(
  const char *phase,
  const BoardTestStats &stats
) {
  Serial.print(phase);
  Serial.print(" mean=");
  Serial.print(stats.mean, 4);
  Serial.print(" std=");
  Serial.print(stats.stddev(), 4);
  Serial.print(" min=");
  Serial.print(stats.minValue, 4);
  Serial.print(" max=");
  Serial.print(stats.maxValue, 4);
  Serial.print(" pkpk=");
  Serial.print(stats.pkpk(), 4);
}

static void boardTestCollectCurrentStats(
  CurrentSense &currentSense,
  BoardTestCurrentStats &stats
) {
  for (uint16_t i = 0; i < BOARD_TEST_CURRENT_SAMPLES; i++) {
    const PhaseCurrent_s current = currentSense.getPhaseCurrents();
    stats.a.add(current.a);
    stats.b.add(current.b);
    stats.c.add(current.c);
    delayMicroseconds(BOARD_TEST_CURRENT_SAMPLE_US);
  }
}

static bool boardTestCurrentNoise(
  const char *label,
  CurrentSense &currentSense,
  bool currentFeedbackOk
) {
  if (!currentFeedbackOk || !currentSense.initialized) {
    Serial.print(label);
    Serial.println(" current: SKIP, ADC/current sense not initialized.");
    return false;
  }

  BoardTestCurrentStats stats;
  boardTestCollectCurrentStats(currentSense, stats);
  const bool ok = boardTestCurrentStatsOk(stats);

  Serial.print(label);
  Serial.print(" current zero/noise ");
  printBoardTestResult(ok);
  Serial.print("  ");
  boardTestPrintCurrentStats("A", stats.a);
  Serial.println();
  Serial.print("  ");
  boardTestPrintCurrentStats("B", stats.b);
  Serial.println();
  Serial.print("  ");
  boardTestPrintCurrentStats("C", stats.c);
  Serial.println();
  return ok;
}

static void boardTestRawAdcSnapshot(bool anyCurrentFeedbackOk) {
  if (!anyCurrentFeedbackOk) {
    return;
  }

  const BU79100QuadSample sample = currentAdc.read();
  Serial.print("ADC raw snapshot M0[A,C] M1[A,C]=");
  Serial.print(sample.raw[0]);
  Serial.print(",");
  Serial.print(sample.raw[1]);
  Serial.print(" ");
  Serial.print(sample.raw[2]);
  Serial.print(",");
  Serial.println(sample.raw[3]);
}

static bool boardTestPrepareOpenLoopMotor(
  BLDCMotor &motor,
  DRV8316Driver3PWM &driver,
  float voltageLimit
) {
  motor.linkDriver(&driver);
  motor.sensor = nullptr;
  motor.current_sense = nullptr;
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::velocity_openloop;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.phase_resistance = NOT_SET;
  motor.phase_inductance = NOT_SET;
  motor.KV_rating = NOT_SET;
  motor.sensor_direction = Direction::CW;
  motor.zero_electric_angle = 0.0f;
  motor.sensor_offset = 0.0f;
  motor.voltage_limit = _constrain(voltageLimit, 0.0f, driver.voltage_limit);
  motor.current_limit = GM3506_PEAK_CURRENT_A;
  motor.velocity_limit = 50.0f;
  motor.motion_downsample = 0;
  motor.shaft_angle = 0.0f;
  motor.shaft_velocity = 0.0f;
  motor.target = 0.0f;
  return motor.init() != 0;
}

static void boardTestStopOpenLoopMotor(BLDCMotor &motor, DRV8316Driver3PWM &driver) {
  if (motor.driver != nullptr && motor.driver->initialized) {
    motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
    motor.disable();
  }
  if (driver.initialized) {
    driver.setPwm(0.0f, 0.0f, 0.0f);
    driver.disable();
  }
}

static void boardTestPrintLiveCurrent(
  const char *label,
  CurrentSense &currentSense,
  bool currentFeedbackOk,
  uint32_t elapsedMs
) {
  if (!currentFeedbackOk || !currentSense.initialized) {
    return;
  }

  const PhaseCurrent_s current = currentSense.getPhaseCurrents();
  Serial.print(label);
  Serial.print(" t=");
  Serial.print((float)elapsedMs * 0.001f, 2);
  Serial.print("s i=");
  Serial.print(current.a, 3);
  Serial.print(",");
  Serial.print(current.b, 3);
  Serial.print(",");
  Serial.println(current.c, 3);
}

static void boardTestOpenLoopVelocity(
  const char *label,
  BLDCMotor &motor,
  DRV8316Driver3PWM &driver,
  CurrentSense &currentSense,
  bool currentFeedbackOk
) {
  Serial.println();
  Serial.print(label);
  Serial.println(" open-loop velocity test.");
  Serial.println("Motor will spin without encoder feedback. Keep voltage low.");

  const float voltage = readFloatWithDefault(
    "Voltage limit [V]",
    BOARD_TEST_OPENLOOP_DEFAULT_VOLTAGE,
    0.0f,
    BOARD_TEST_OPENLOOP_MAX_VOLTAGE
  );
  const float velocity = readFloatWithDefault(
    "Velocity target [rad/s]",
    BOARD_TEST_OPENLOOP_DEFAULT_VELOCITY,
    -50.0f,
    50.0f
  );
  const uint32_t durationMs =
    readUint32WithDefault("Duration [ms]", BOARD_TEST_OPENLOOP_DEFAULT_MS);

  if (!serialConfirm("Start open-loop spin?", false)) {
    return;
  }
  if (!boardTestPrepareOpenLoopMotor(motor, driver, voltage)) {
    Serial.print(label);
    Serial.println(" open-loop init failed.");
    boardTestStopOpenLoopMotor(motor, driver);
    return;
  }

  Serial.println("Running. Send q then Enter to stop early.");
  const uint32_t startMs = millis();
  uint32_t lastPrintMs = 0;
  while ((millis() - startMs) < durationMs) {
    while (Serial.available() > 0) {
      const int value = Serial.read();
      if (value == 'q' || value == 'Q') {
        boardTestStopOpenLoopMotor(motor, driver);
        clearSerialInput();
        Serial.println("Stopped.");
        return;
      }
    }

    motor.move(velocity);

    const uint32_t elapsedMs = millis() - startMs;
    if ((elapsedMs - lastPrintMs) >= 250) {
      lastPrintMs = elapsedMs;
      boardTestPrintLiveCurrent(label, currentSense, currentFeedbackOk, elapsedMs);
    }
    delayMicroseconds(BOARD_TEST_OPENLOOP_LOOP_US);
  }

  boardTestStopOpenLoopMotor(motor, driver);
  clearSerialInput();
  Serial.println("Done.");
}

static void boardTestPrintMenu() {
  Serial.println();
  Serial.println("Board test commands:");
  Serial.println("  d - test DRV communication");
  Serial.println("  e - test encoder communication");
  Serial.println("  c - test current zero/noise");
  Serial.println("  0 - open-loop velocity M0");
  Serial.println("  1 - open-loop velocity M1");
  Serial.println("  h - print this menu");
  Serial.println("  q - halt");
}

static void boardTestDrvCommand() {
  boardTestDrvCommunication("M0", GPIO_M0_DRV_CS, driver0);
  boardTestDrvCommunication("M1", GPIO_M1_DRV_CS, driver1);
}

static void boardTestEncoderCommand() {
  boardTestEncoderCommunication("M0", encoder0);
  boardTestEncoderCommunication("M1", encoder1);
}

static void boardTestCurrentCommand(const MotorHardwareStatus &hardware) {
  driver0.setPwm(0.0f, 0.0f, 0.0f);
  driver1.setPwm(0.0f, 0.0f, 0.0f);
  boardTestRawAdcSnapshot(hardware.currentFeedback0Ok || hardware.currentFeedback1Ok);
  boardTestCurrentNoise("M0", currentSense0, hardware.currentFeedback0Ok);
  boardTestCurrentNoise("M1", currentSense1, hardware.currentFeedback1Ok);
}

static void runBoardTestMode() {
  clearSerialInput();

  Serial.println();
  Serial.println("Dual PMSM board test mode");
  Serial.println("Binary protocol is disabled in this boot mode.");
  Serial.println("Use this mode for bring-up and production checks.");
  Serial.println();

  Serial.println("Initializing motor hardware...");
  const MotorHardwareStatus hardware = initializeMotorHardware();
  initializeRuntimeBusVoltage(hardware.busVoltage, micros());

  Serial.print("VBUS=");
  Serial.print(hardware.busVoltage, 3);
  Serial.println(" V");
  Serial.print("ADC current feedback: M0=");
  Serial.print(hardware.currentFeedback0Ok ? "OK" : "FAIL");
  Serial.print(" M1=");
  Serial.println(hardware.currentFeedback1Ok ? "OK" : "FAIL");
  Serial.print("Startup encoder health: M0=");
  Serial.print(hardware.encoder0Allowed ? "OK" : "FAIL");
  Serial.print(" M1=");
  Serial.println(hardware.encoder1Allowed ? "OK" : "FAIL");
  Serial.println();

  boardTestDrvCommand();
  boardTestEncoderCommand();
  boardTestCurrentCommand(hardware);
  boardTestPrintMenu();

  char line[24];
  while (true) {
    Serial.println();
    Serial.print("test> ");
    readSerialLine(line, sizeof(line));
    const char command = firstCommandChar(line);

    if (command == 'd' || command == 'D') {
      boardTestDrvCommand();
    } else if (command == 'e' || command == 'E') {
      boardTestEncoderCommand();
    } else if (command == 'c' || command == 'C') {
      boardTestCurrentCommand(hardware);
    } else if (command == '0') {
      boardTestOpenLoopVelocity(
        "M0",
        motor0,
        driver0,
        currentSense0,
        hardware.currentFeedback0Ok
      );
    } else if (command == '1') {
      boardTestOpenLoopVelocity(
        "M1",
        motor1,
        driver1,
        currentSense1,
        hardware.currentFeedback1Ok
      );
    } else if (command == 'h' || command == 'H' || command == '\0') {
      boardTestPrintMenu();
    } else if (command == 'q' || command == 'Q') {
      boardTestStopOpenLoopMotor(motor0, driver0);
      boardTestStopOpenLoopMotor(motor1, driver1);
      Serial.println("Board test halted. Reset the board to leave this mode.");
      while (true) {
        delay(1000);
      }
    } else {
      Serial.println("Unknown command. Type h for help.");
    }
  }
}
