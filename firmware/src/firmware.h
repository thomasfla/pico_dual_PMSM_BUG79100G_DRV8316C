#pragma once

#include <Arduino.h>
#include <SimpleFOC.h>
#include "board_config.h"
#include "board_pwm.h"
#include "checked_encoder.h"
#include "current_feedback.h"
#include "calibration_store.h"
#include "status_led.h"

constexpr uint8_t CONTROL_REFERENCE_FLAG_M0 = 1u << 0;
constexpr uint8_t CONTROL_REFERENCE_FLAG_M1 = 1u << 1;

enum class SerialBootMode : uint8_t {
  Control,
  Calibration,
  BoardTest,
};

struct PositionHoldState {
  float targetPosition = 0.0f;
  float targetVelocity = 0.0f;
  float feedforwardCurrent = 0.0f;
  float iqCommand = 0.0f;
};

struct RuntimeMotorState {
  float position = 0.0f;
  float velocity = 0.0f;
  float velocityHighFrequency = 0.0f;
  float iq = 0.0f;
  float iqTarget = 0.0f;
  bool ready = false;
};

struct TelemetryVelocityFilterState {
  float value = 0.0f;
  uint32_t tUs = 0;
  bool initialized = false;
};

struct RuntimeControlState {
  uint32_t tUs = 0;
  uint32_t latestCommandIndex = 0;
  float controlLoopUs = 0.0f;
  uint8_t flags = 0;
  RuntimeMotorState m0;
  RuntimeMotorState m1;
};

struct MotorReferenceCommand {
  float targetPosition = 0.0f;
  float targetVelocity = 0.0f;
  float feedforwardCurrent = 0.0f;
  float kp = 0.0f;
  float kd = 0.0f;
};

struct ControlReferenceCommand {
  uint32_t tUs = 0;
  uint32_t commandIndex = 0;
  uint16_t timeoutMs = 0;
  uint8_t flags = 0;
  MotorReferenceCommand m0;
  MotorReferenceCommand m1;
};

struct MotorHardwareStatus {
  float busVoltage = 0.0f;
  bool encoder0Allowed = false;
  bool encoder1Allowed = false;
  bool currentFeedback0Ok = false;
  bool currentFeedback1Ok = false;
};


extern BoardMotorDriver driver0, driver1;
extern BLDCMotor motor0, motor1;
extern CheckedAS5048ASensor encoder0, encoder1;
extern PositionHoldConfig runtimeConfig0, runtimeConfig1;
extern CalibrationSettings activeCalibrationSettings;
extern StatusLed statusLed;
extern bool usbPendingByteValid;
extern uint8_t usbPendingByte;

void deselectSpiSlaves();
Direction directionFromSign(int8_t sign);
int8_t signFromDirection(Direction direction);
void applyMotorCalibration(
  PositionHoldConfig &config,
  const MotorCalibrationSettings &calibration
);
void applyCalibrationSettings(const CalibrationSettings &settings);
void loadOrCreateCalibrationSettings();
bool configureMotor(
  BLDCMotor &motor,
  DRV8316Driver3PWM &driver,
  CurrentSense &currentSense,
  Sensor &sensor,
  const PositionHoldConfig &config
);
void setIqTarget(BLDCMotor &motor, float iq);
void initializeRuntimeBusVoltage(float measuredBusVoltage, uint32_t nowUs);
MotorHardwareStatus initializeMotorHardware();
bool encoderOcf(uint16_t diagnostics);
bool encoderCof(uint16_t diagnostics);
bool encoderCompLow(uint16_t diagnostics);
bool encoderCompHigh(uint16_t diagnostics);
bool encoderDiagnosticsHealthy(const EncoderRegisterDiagnostics &regs);
void publishRuntimeControlState(const RuntimeControlState &state);
bool readLatestRuntimeControlState(RuntimeControlState &state);
bool readLatestControlReferenceCommand(
  ControlReferenceCommand &command,
  uint32_t &sequence
);
void writeUsbStatePacketIfDue(const RuntimeControlState &state);
void readUsbCommandPackets();
void clearSerialInput();
size_t readSerialLine(char *buffer, size_t length);
char firstCommandChar(const char *line);
bool serialConfirm(const char *prompt, bool defaultYes);
uint32_t readUint32WithDefault(const char *prompt, uint32_t currentValue);
SerialBootMode serialBootModeRequested();
void runCalibrationWizard();
void controlSetup();
void controlStep();
void serviceEmergencyStop();
void discardControlReferenceCommands(uint32_t &sequence);
void runBoardTestMode();
