#include "firmware.h"
#include "emergency_stop.h"

bool core1_disable_systick = true;
bool core1_separate_stack = true;

StatusLed statusLed(
  GPIO_PICO_LED,
  STATUS_LED_STARTUP_BLINK_US,
  STATUS_LED_TIMEOUT_BLINK_US,
  STATUS_LED_CONTROLLED_BLINK_US
);
static volatile bool interfaceCoreReady = false;
static volatile bool calibrationModeActive = false;
static volatile bool boardTestModeActive = false;
static void sharedMemoryBarrier() { __asm__ volatile("dmb sy" ::: "memory"); }

void setup() {
  pinMode(GPIO_ESTOP, INPUT_PULLUP);
  Serial.begin(115200);
  statusLed.begin();
  deselectSpiSlaves();

  const uint32_t serialStartMs = millis();
  while (!Serial && (millis() - serialStartMs) < SERIAL_STARTUP_WAIT_MS) {
    statusLed.serviceStartup();
  }

  loadOrCreateCalibrationSettings();

  const SerialBootMode bootMode = serialBootModeRequested();
  if (bootMode == SerialBootMode::Calibration) {
    calibrationModeActive = true;
    sharedMemoryBarrier();
    runCalibrationWizard();
    while (true) {
      statusLed.delayStartup(1000);
    }
  }
  if (bootMode == SerialBootMode::BoardTest) {
    boardTestModeActive = true;
    sharedMemoryBarrier();
    runBoardTestMode();
    while (true) {
      statusLed.delayStartup(1000);
    }
  }

  sharedMemoryBarrier();
  interfaceCoreReady = true;
}

void loop() {
  emergency_stop::poll();
  readUsbCommandPackets();

  RuntimeControlState runtimeState;
  const bool hasRuntimeState = readLatestRuntimeControlState(runtimeState);

  if (hasRuntimeState) {
    statusLed.noteNormalStarted();
    writeUsbStatePacketIfDue(runtimeState);
  }
  statusLed.serviceInterface();

  if (INTERFACE_IDLE_US > 0) {
    delayMicroseconds(INTERFACE_IDLE_US);
  }
}

void setup1() {
  while (!interfaceCoreReady) {
    if (calibrationModeActive || boardTestModeActive) {
      while (true) {
        emergency_stop::poll();
        delayMicroseconds(20);
      }
    }
    delayMicroseconds(10);
  }
  controlSetup();
}

void loop1() {
  while (true) {
    controlStep();
  }
}
