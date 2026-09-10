#include "firmware.h"
#include "hardware/adc.h"
#include "control_math.h"
#include "emergency_stop.h"

static_assert(VBUS_ADC_BITS == 12, "SDK ADC reads return 12-bit samples");
static_assert(
  GPIO_VBUS_SENSE >= __FIRSTANALOGGPIO,
  "VBUS sense pin is below Arduino ADC GPIO range"
);
static_assert(
  GPIO_VBUS_SENSE <= __GPIOCNT,
  "VBUS sense pin is outside Arduino ADC GPIO range"
);
static constexpr uint8_t VBUS_ADC_INPUT = GPIO_VBUS_SENSE - __FIRSTANALOGGPIO;

BoardMotorDriver driver0(0, GPIO_M0_DRV_CS, GPIO_DRV_Mx_nFAULT);
BoardMotorDriver driver1(1, GPIO_M1_DRV_CS, GPIO_DRV_Mx_nFAULT);

BLDCMotor motor0(GM3506_POLE_PAIRS);
BLDCMotor motor1(GM3506_POLE_PAIRS);
CheckedAS5048ASensor encoder0(GPIO_M0_ENC_CS);
CheckedAS5048ASensor encoder1(GPIO_M1_ENC_CS);
static uint32_t lastRuntimePublishUs = 0;
static PositionHoldState control0;
static PositionHoldState control1;
PositionHoldConfig runtimeConfig0 = MOTOR0_CONFIG;
PositionHoldConfig runtimeConfig1 = MOTOR1_CONFIG;
CalibrationSettings activeCalibrationSettings;
static bool motor0Ready = false;
static bool motor1Ready = false;
static bool runtimeFault = false;
// Encoded in state flags bits 3..7; keep packet size/version unchanged.
enum class ControlFault : uint8_t {
  None = 0, Driver = 1, CurrentFeedback = 2, AdcFrameGap = 3,
  SampleTiming = 4, Encoder0 = 5, Encoder1 = 6,
  NonFiniteControl = 7, PwmDeadline = 8, EmergencyStop = 9,
};
static ControlFault runtimeFaultCause = ControlFault::None;
static bool motorHardwareInitialized = false;
static bool normalControlMode = false;
static uint8_t readyBeforeEmergencyStop = 0;
static uint32_t lastAppliedCommandSequence = 0;
static uint32_t controlLoopCounter = 0;
static uint32_t lastControlFrame = 0;
static uint32_t lastRuntimePublishLoopCounter = 0;
static uint32_t latestAppliedCommandIndex = 0;
static uint32_t lastCommandApplyUs = 0;
static uint32_t commandTimeoutUs = 0;
static float lastControlLoopUs = 0.0f;
static bool commandTorqueEnabled = false;
static bool commandMotor0Enabled = false;
static bool commandMotor1Enabled = false;
static TelemetryVelocityFilterState telemetryVelocity0;
static TelemetryVelocityFilterState telemetryVelocity1;
static float filteredBusVoltage = SUPPLY_VOLTAGE_FALLBACK;
static bool filteredBusVoltageInitialized = false;
static uint32_t lastBusVoltageUpdateUs = 0;
static bool runtimeBusVoltageConversionPending = false;

void deselectSpiSlaves() {
  const uint8_t csPins[] = {
    GPIO_M0_ENC_CS,
    GPIO_M1_ENC_CS,
    GPIO_M0_DRV_CS,
    GPIO_M1_DRV_CS,
  };

  for (uint8_t pin : csPins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
  }
}

static void configureSpiPins() {
  SPI.setRX(GPIO_SPI0_MISO);
  SPI.setSCK(GPIO_SPI0_CLK);
  SPI.setTX(GPIO_SPI0_MOSI);
  gpio_set_function(GPIO_SPI0_MISO, GPIO_FUNC_SPI);
  gpio_set_function(GPIO_SPI0_CLK, GPIO_FUNC_SPI);
  gpio_set_function(GPIO_SPI0_MOSI, GPIO_FUNC_SPI);
  gpio_pull_up(GPIO_SPI0_MISO);
  sharedSpiMarkUnknown();
}

static void configureBusVoltageSense() {
  adc_init();
  adc_run(false);
  adc_set_round_robin(0);
  adc_set_temp_sensor_enabled(false);
  // Arduino-Pico may leave the SDK ADC base-pin define at the RP2350B default.
  gpio_set_function(GPIO_VBUS_SENSE, GPIO_FUNC_NULL);
  gpio_disable_pulls(GPIO_VBUS_SENSE);
  gpio_set_input_enabled(GPIO_VBUS_SENSE, false);
  adc_select_input(VBUS_ADC_INPUT);
  runtimeBusVoltageConversionPending = false;
}

static float busVoltageFromAdcCounts(float rawCounts) {
  const float adcVoltage = rawCounts * CURRENT_SENSE_VREF / VBUS_ADC_MAX_COUNTS;
  return adcVoltage * VBUS_DIVIDER_RATIO;
}

static uint16_t readBusVoltageAdcBlocking() {
  adc_select_input(VBUS_ADC_INPUT);
  return adc_read();
}

static void startRuntimeBusVoltageConversion() {
  if ((adc_hw->cs & ADC_CS_READY_BITS) == 0) {
    return;
  }

  adc_select_input(VBUS_ADC_INPUT);
  hw_set_bits(&adc_hw->cs, ADC_CS_START_ONCE_BITS);
  runtimeBusVoltageConversionPending = true;
}

static bool readRuntimeBusVoltageIfReady(float &measuredBusVoltage) {
  if (!runtimeBusVoltageConversionPending) {
    return false;
  }
  if ((adc_hw->cs & ADC_CS_READY_BITS) == 0) {
    return false;
  }

  runtimeBusVoltageConversionPending = false;
  measuredBusVoltage = busVoltageFromAdcCounts((float)(uint16_t)adc_hw->result);
  return true;
}

static float readBusVoltage() {
  uint32_t sum = 0;

  configureBusVoltageSense();
  delay(2);

  for (uint16_t i = 0; i < VBUS_STARTUP_SAMPLES; i++) {
    sum += readBusVoltageAdcBlocking();
    delayMicroseconds(50);
  }

  const float rawCounts = (float)sum / VBUS_STARTUP_SAMPLES;
  return busVoltageFromAdcCounts(rawCounts);
}

Direction directionFromSign(int8_t sign) {
  if (sign > 0) {
    return Direction::CW;
  }
  if (sign < 0) {
    return Direction::CCW;
  }
  return Direction::UNKNOWN;
}

int8_t signFromDirection(Direction direction) {
  if (direction == Direction::CW) {
    return 1;
  }
  if (direction == Direction::CCW) {
    return -1;
  }
  return 0;
}

void applyMotorCalibration(
  PositionHoldConfig &config,
  const MotorCalibrationSettings &calibration
) {
  config.sensorDirectionSign = calibration.sensorDirectionSign;
  config.zeroElectricAngle = calibration.zeroElectricAngle;
  config.sensorOffset = config.sensorDirectionSign == 0 ? calibration.sensorOffset :
    control_math::singleTurnOffset(calibration.sensorOffset, config.sensorDirectionSign);
}

void applyCalibrationSettings(const CalibrationSettings &settings) {
  runtimeConfig0 = MOTOR0_CONFIG;
  runtimeConfig1 = MOTOR1_CONFIG;
  applyMotorCalibration(runtimeConfig0, settings.motor[0]);
  applyMotorCalibration(runtimeConfig1, settings.motor[1]);
}

void loadOrCreateCalibrationSettings() {
  beginCalibrationStore();
  if (!loadCalibrationSettings(activeCalibrationSettings)) {
    activeCalibrationSettings = makeDefaultCalibrationSettings();
    saveCalibrationSettings(activeCalibrationSettings);
  }
  applyCalibrationSettings(activeCalibrationSettings);
}

static bool busVoltageMeasurementValid(float measuredVoltage) {
  if (measuredVoltage > 4.0f && measuredVoltage < 30.0f) {
    return true;
  }
  return false;
}

static float usableSupplyVoltage(float measuredVoltage) {
  if (busVoltageMeasurementValid(measuredVoltage)) {
    return measuredVoltage;
  }
  return SUPPLY_VOLTAGE_FALLBACK;
}

static float clampVoltageLimitToBus(float requestedLimit, float supplyVoltage) {
  return _constrain(
    requestedLimit,
    0.0f,
    supplyVoltage * DRIVER_VOLTAGE_LIMIT_BUS_FRACTION
  );
}

static bool configureDriver(BoardMotorDriver &driver, float measuredBusVoltage) {
  driver.voltage_power_supply = usableSupplyVoltage(measuredBusVoltage);
  driver.voltage_limit = clampVoltageLimitToBus(DRIVER_VOLTAGE_LIMIT, driver.voltage_power_supply);
  driver.pwm_frequency = PWM_FREQUENCY;

  driver.init(&SPI);
  delayMicroseconds(5);

  driver.clearFault();
  delayMicroseconds(5);

  driver.setRegistersLocked(false);
  delayMicroseconds(5);

  driver.setPWMMode(DRV8316_PWMMode::PWM3_Mode);
  delayMicroseconds(5);

  driver.setSDOMode(DRV8316_SDOMode::SDOMode_PushPull);
  delayMicroseconds(5);

  driver.setSlew(DRV8316_Slew::Slew_25Vus);
  delayMicroseconds(5);

  driver.setOvertemperatureReporting(true);
  delayMicroseconds(5);

  driver.setSPIFaultReporting(true);
  delayMicroseconds(5);

  driver.setOvervoltageProtection(true);
  delayMicroseconds(5);

  driver.setOvervoltageLevel(DRV8316_OVP::OVP_SEL_32V);
  delayMicroseconds(5);

  driver.setOCPMode(DRV8316_OCPMode::Latched_Fault);
  delayMicroseconds(5);

  driver.setOCPLevel(DRV8316_OCPLevel::Curr_16A);
  delayMicroseconds(5);

  driver.setOCPRetryTime(DRV8316_OCPRetry::Retry5ms);
  delayMicroseconds(5);

  driver.setOCPDeglitchTime(DRV8316_OCPDeglitch::Deglitch_1us1);
  delayMicroseconds(5);

  driver.setOCPClearInPWMCycleChange(false);
  delayMicroseconds(5);

  // DRV8316C raw CSA_GAIN=10b is 0.6 V/A; the enum name is for another variant.
  driver.setCurrentSenseGain(DRV8316_CSAGain::Gain_0V25);
  delayMicroseconds(5);

  driver.setDriverOffEnabled(emergency_stop::active());
  delayMicroseconds(5);

  driver.clearFault();
  const auto status = driver.getStatus();
  const bool ok = driver.getPWMMode() == DRV8316_PWMMode::PWM3_Mode &&
    driver.getCurrentSenseGain() == DRV8316_CSAGain::Gain_0V25 &&
    !status.status.FAULT && !status.status.SPI_FLT;
  if (!ok) driver.setDriverOffEnabled(true);
  driver.initialized = ok;
  return ok;
}

static void applyMotorVoltageLimit(BLDCMotor &motor, bool ready) {
  if (!ready || motor.driver == nullptr) {
    return;
  }

  const float voltageLimit = _constrain(
    CURRENT_FOC_VOLTAGE_LIMIT,
    0.0f,
    motor.driver->voltage_limit
  );
  motor.voltage_limit = voltageLimit;
  if (motor.current_sense != nullptr) {
    motor.PID_current_q.limit = voltageLimit;
    motor.PID_current_d.limit = voltageLimit;
  }
}

static void applyRuntimeBusVoltage(float busVoltage) {
  const float supplyVoltage = usableSupplyVoltage(busVoltage);
  filteredBusVoltage = supplyVoltage;

  driver0.updateSupplyVoltage(supplyVoltage);
  driver1.updateSupplyVoltage(supplyVoltage);
  driver0.voltage_limit = clampVoltageLimitToBus(DRIVER_VOLTAGE_LIMIT, supplyVoltage);
  driver1.voltage_limit = clampVoltageLimitToBus(DRIVER_VOLTAGE_LIMIT, supplyVoltage);

  applyMotorVoltageLimit(motor0, motor0Ready);
  applyMotorVoltageLimit(motor1, motor1Ready);
}

void initializeRuntimeBusVoltage(float measuredBusVoltage, uint32_t nowUs) {
  filteredBusVoltageInitialized = true;
  lastBusVoltageUpdateUs = nowUs;
  applyRuntimeBusVoltage(measuredBusVoltage);
}

static void updateRuntimeBusVoltageIfDue(uint32_t nowUs) {
  if (!filteredBusVoltageInitialized) {
    initializeRuntimeBusVoltage(SUPPLY_VOLTAGE_FALLBACK, nowUs);
  }

  float measuredBusVoltage = 0.0f;
  if (runtimeBusVoltageConversionPending) {
    if (!readRuntimeBusVoltageIfReady(measuredBusVoltage)) {
      return;
    }
    if (!busVoltageMeasurementValid(measuredBusVoltage)) {
      lastBusVoltageUpdateUs = nowUs;
      return;
    }

    const float dt = (float)(nowUs - lastBusVoltageUpdateUs) * 1.0e-6f;
    lastBusVoltageUpdateUs = nowUs;
    if (dt <= 0.0f) {
      return;
    }

    const float alpha = dt / (VBUS_RUNTIME_FILTER_TF + dt);
    filteredBusVoltage += alpha * (measuredBusVoltage - filteredBusVoltage);
    applyRuntimeBusVoltage(filteredBusVoltage);
    return;
  }

  if ((nowUs - lastBusVoltageUpdateUs) < VBUS_RUNTIME_UPDATE_INTERVAL_US) {
    return;
  }

  startRuntimeBusVoltageConversion();
}

bool encoderOcf(uint16_t diagnostics) {
  return ((diagnostics >> 8) & 0x1) != 0;
}

bool encoderCof(uint16_t diagnostics) {
  return ((diagnostics >> 9) & 0x1) != 0;
}

bool encoderCompLow(uint16_t diagnostics) {
  return ((diagnostics >> 10) & 0x1) != 0;
}

bool encoderCompHigh(uint16_t diagnostics) {
  return ((diagnostics >> 11) & 0x1) != 0;
}

bool encoderDiagnosticsHealthy(const EncoderRegisterDiagnostics &regs) {
  if (!regs.magnitudeOk || !regs.diagnosticsOk) {
    return false;
  }

  const bool magnitudeInRange =
    regs.magnitude >= ENCODER_MAG_MIN && regs.magnitude <= ENCODER_MAG_MAX;
  return magnitudeInRange &&
    encoderOcf(regs.diagnostics) &&
    !encoderCof(regs.diagnostics) &&
    !encoderCompLow(regs.diagnostics) &&
    !encoderCompHigh(regs.diagnostics);
}

static bool fastInitMotor(BLDCMotor &motor) {
  if (motor.driver == nullptr || !motor.driver->initialized) {
    motor.motor_status = FOCMotorStatus::motor_init_failed;
    return false;
  }

  motor.motor_status = FOCMotorStatus::motor_initializing;
  if (motor.voltage_limit > motor.driver->voltage_limit) {
    motor.voltage_limit = motor.driver->voltage_limit;
  }
  if (motor.voltage_sensor_align > motor.voltage_limit) {
    motor.voltage_sensor_align = motor.voltage_limit;
  }

  if (motor.current_sense != nullptr) {
    motor.PID_current_q.limit = motor.voltage_limit;
    motor.PID_current_d.limit = motor.voltage_limit;
  }
  motor.PID_velocity.limit = motor.current_limit;
  motor.P_angle.limit = motor.velocity_limit;

  motor.enable();
  motor.motor_status = FOCMotorStatus::motor_uncalibrated;
  return true;
}

bool configureMotor(
  BLDCMotor &motor,
  DRV8316Driver3PWM &driver,
  CurrentSense &currentSense,
  Sensor &sensor,
  const PositionHoldConfig &config
) {
  motor.linkDriver(&driver);
  motor.linkCurrentSense(&currentSense);
  motor.linkSensor(&sensor);

  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::torque;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  const float voltageLimit = _constrain(CURRENT_FOC_VOLTAGE_LIMIT, 0.0f, driver.voltage_limit);
  motor.phase_resistance = GM3506_PHASE_RESISTANCE_OHM;
  motor.phase_inductance = GM3506_PHASE_INDUCTANCE_H;
  motor.sensor_direction = directionFromSign(config.sensorDirectionSign);
  motor.zero_electric_angle = config.zeroElectricAngle;
  motor.sensor_offset = config.sensorOffset;
  motor.KV_rating = NOT_SET;
  motor.voltage_limit = voltageLimit;
  motor.current_limit = config.iqLimit;
  motor.voltage_sensor_align = POSITION_SENSOR_ALIGN_VOLTAGE;
  motor.LPF_velocity.Tf = POSITION_VELOCITY_FILTER_TF;
  motor.LPF_angle.Tf = 0.0f;
  motor.PID_current_q.P = CURRENT_CONTROL_P;
  motor.PID_current_q.I = CURRENT_CONTROL_I;
  motor.PID_current_q.D = CURRENT_CONTROL_D;
  motor.PID_current_q.output_ramp = CURRENT_CONTROL_RAMP;
  motor.PID_current_q.limit = voltageLimit;
  motor.PID_current_d.P = CURRENT_CONTROL_P;
  motor.PID_current_d.I = CURRENT_CONTROL_I;
  motor.PID_current_d.D = CURRENT_CONTROL_D;
  motor.PID_current_d.output_ramp = CURRENT_CONTROL_RAMP;
  motor.PID_current_d.limit = voltageLimit;
  motor.LPF_current_q.Tf = CURRENT_CONTROL_FILTER_TF;
  motor.LPF_current_d.Tf = CURRENT_CONTROL_FILTER_TF;

  return fastInitMotor(motor);
}

static bool startClosedLoopMotor(BLDCMotor &motor, PositionHoldState &control) {
  const int focOk = motor.initFOC();
  if (focOk) {
    control.targetPosition = motor.shaft_angle;
    control.targetVelocity = 0.0f;
    control.feedforwardCurrent = 0.0f;
    control.iqCommand = 0.0f;
    motor.target = 0.0f;
    motor.current_sp = 0.0f;
    return true;
  } else {
    motor.disable();
    return false;
  }
}

void setIqTarget(BLDCMotor &motor, float iq) {
  motor.target = iq;
  motor.current_sp = iq;
}

static void updateMotorKinematics(BLDCMotor &motor) {
  motor.shaft_angle = motor.shaftAngle();
  motor.shaft_velocity = motor.shaftVelocity();
}

static float constrainFinite(float value, float minValue, float maxValue, float fallback) {
  if (!isfinite(value)) {
    return fallback;
  }
  return _constrain(value, minValue, maxValue);
}

static void applyMotorReferenceCommand(
  const MotorReferenceCommand &command,
  PositionHoldState &control,
  PositionHoldConfig &config
) {
  if (isfinite(command.targetPosition)) {
    control.targetPosition = command.targetPosition;
  }
  control.targetVelocity = constrainFinite(
    command.targetVelocity,
    -POSITION_TARGET_VELOCITY_LIMIT_RAD_S,
    POSITION_TARGET_VELOCITY_LIMIT_RAD_S,
    control.targetVelocity
  );
  control.feedforwardCurrent = constrainFinite(
    command.feedforwardCurrent,
    -config.iqLimit,
    config.iqLimit,
    control.feedforwardCurrent
  );
  config.kp = constrainFinite(
    command.kp,
    POSITION_PD_KP_MIN_A_PER_RAD,
    POSITION_PD_KP_MAX_A_PER_RAD,
    config.kp
  );
  config.kd = constrainFinite(
    command.kd,
    POSITION_PD_KD_MIN_A_PER_RAD_PER_S,
    POSITION_PD_KD_MAX_A_PER_RAD_PER_S,
    config.kd
  );
}

static void applyControlReferenceCommandIfAvailable() {
  ControlReferenceCommand command;
  uint32_t sequence = lastAppliedCommandSequence;

  if (!readLatestControlReferenceCommand(command, sequence)) {
    return;
  }
  lastAppliedCommandSequence = sequence;
  latestAppliedCommandIndex = command.commandIndex;
  if (runtimeFault) return;
  lastCommandApplyUs = command.tUs;
  commandTimeoutUs = (uint32_t)command.timeoutMs * 1000u;
  commandTorqueEnabled = true;
  commandMotor0Enabled = (command.flags & CONTROL_REFERENCE_FLAG_M0) != 0;
  commandMotor1Enabled = (command.flags & CONTROL_REFERENCE_FLAG_M1) != 0;

  if (commandMotor0Enabled) {
    applyMotorReferenceCommand(command.m0, control0, runtimeConfig0);
  }
  if (commandMotor1Enabled) {
    applyMotorReferenceCommand(command.m1, control1, runtimeConfig1);
  }
}

static bool commandTorqueAllowed(uint32_t nowUs) {
  if (!commandTorqueEnabled) {
    return false;
  }
  if (commandTimeoutUs == 0) {
    return true;
  }
  if ((nowUs - lastCommandApplyUs) <= commandTimeoutUs) {
    return true;
  }

  commandTorqueEnabled = false;
  commandMotor0Enabled = false;
  commandMotor1Enabled = false;
  control0.iqCommand = 0.0f;
  control1.iqCommand = 0.0f;
  return false;
}

static float runPositionHoldPd(
  BLDCMotor &motor,
  PositionHoldState &control,
  const PositionHoldConfig &config
) {
  const float angle = motor.shaft_angle;
  const float velocity = motor.shaft_velocity;
  const float angleError = control.targetPosition - angle;
  const float velocityError = control.targetVelocity - velocity;
  const float iq = config.kp * angleError +
    config.kd * velocityError +
    control.feedforwardCurrent;

  control.iqCommand = _constrain(iq, -config.iqLimit, config.iqLimit);
  return control.iqCommand;
}

static void settleStartupTargets(uint32_t settleMs) {
  if (!motor0Ready && !motor1Ready) {
    return;
  }

  const uint32_t startMs = millis();
  while ((millis() - startMs) < settleMs) {
    if (motor0Ready) {
      encoder0.update();
      setIqTarget(motor0, 0.0f);
    }
    if (motor1Ready) {
      encoder1.update();
      setIqTarget(motor1, 0.0f);
    }
    delayMicroseconds(250);
  }

  if (motor0Ready) {
    control0.targetPosition = motor0.shaftAngle();
    control0.targetVelocity = 0.0f;
    control0.feedforwardCurrent = 0.0f;
    control0.iqCommand = 0.0f;
  }
  if (motor1Ready) {
    control1.targetPosition = motor1.shaftAngle();
    control1.targetVelocity = 0.0f;
    control1.feedforwardCurrent = 0.0f;
    control1.iqCommand = 0.0f;
  }
}

static float updateTelemetryVelocityFilter(
  TelemetryVelocityFilterState &filter,
  float input,
  bool ready,
  uint32_t nowUs
) {
  if (!ready || !isfinite(input)) {
    filter.initialized = false;
    filter.value = 0.0f;
    filter.tUs = nowUs;
    return 0.0f;
  }

  if (!filter.initialized) {
    filter.initialized = true;
    filter.value = input;
    filter.tUs = nowUs;
    return filter.value;
  }

  const float dt = (float)(nowUs - filter.tUs) * 1.0e-6f;
  filter.tUs = nowUs;
  if (dt <= 0.0f) {
    return filter.value;
  }

  const float alpha = dt / (TELEMETRY_VELOCITY_FILTER_TF + dt);
  filter.value += alpha * (input - filter.value);
  return filter.value;
}

static RuntimeMotorState makeRuntimeMotorState(
  BLDCMotor &motor,
  const PositionHoldState &control,
  bool ready,
  TelemetryVelocityFilterState &velocityFilter,
  uint32_t nowUs
) {
  RuntimeMotorState state;
  const float highFrequencyVelocity = ready ? motor.shaft_velocity : 0.0f;
  state.position = ready ? motor.shaft_angle : 0.0f;
  state.velocity = updateTelemetryVelocityFilter(
    velocityFilter,
    highFrequencyVelocity,
    ready,
    nowUs
  );
  state.velocityHighFrequency = highFrequencyVelocity;
  state.iq = ready ? motor.current.q : 0.0f;
  state.iqTarget = ready ? control.iqCommand : 0.0f;
  state.ready = ready;
  return state;
}

static RuntimeControlState makeRuntimeControlState() {
  RuntimeControlState state;
  const uint32_t nowUs = micros();
  state.tUs = nowUs;
  state.latestCommandIndex = latestAppliedCommandIndex;
  state.controlLoopUs = runtimeFaultCause == ControlFault::PwmDeadline ?
    float(board_pwm::lastFrameDurationUs()) : lastControlLoopUs;
  state.flags = (motor0Ready ? USB_STATE_FLAG_M0_READY : 0) |
    (motor1Ready ? USB_STATE_FLAG_M1_READY : 0) |
    (runtimeFault ? USB_STATE_FLAG_FAULT : 0) |
    (uint8_t(runtimeFaultCause) << 3);
  state.m0 = makeRuntimeMotorState(
    motor0,
    control0,
    motor0Ready,
    telemetryVelocity0,
    nowUs
  );
  state.m1 = makeRuntimeMotorState(
    motor1,
    control1,
    motor1Ready,
    telemetryVelocity1,
    nowUs
  );
  return state;
}

static void publishRuntimeState() {
  publishRuntimeControlState(makeRuntimeControlState());
}

static bool checkStartupEncoderHealth(
  CheckedAS5048ASensor &encoder,
  EncoderRegisterDiagnostics &regs
) {
  for (uint8_t attempt = 0; attempt < ENCODER_HEALTH_READ_ATTEMPTS; attempt++) {
    regs = encoder.readRegisterDiagnostics();
    if (encoderDiagnosticsHealthy(regs)) {
      return true;
    }
    delayMicroseconds(ENCODER_HEALTH_RETRY_US);
  }
  return false;
}

MotorHardwareStatus initializeMotorHardware() {
  MotorHardwareStatus status;

  emergency_stop::init();
  deselectSpiSlaves();
  pinMode(GPIO_DRV_Mx_nFAULT, INPUT_PULLUP);
  configureSpiPins();

  status.busVoltage = readBusVoltage();

  delay(ENCODER_POWERUP_DELAY_MS);
  encoder0.init();
  encoder1.init();
  EncoderRegisterDiagnostics encoder0Regs;
  EncoderRegisterDiagnostics encoder1Regs;
  const bool encoder0Healthy = checkStartupEncoderHealth(encoder0, encoder0Regs);
  const bool encoder1Healthy = checkStartupEncoderHealth(encoder1, encoder1Regs);

  board_pwm::init();
  const bool driver0Ok = configureDriver(driver0, status.busVoltage);
  const bool driver1Ok = configureDriver(driver1, status.busVoltage);
  pinMode(GPIO_DRV_Mx_nFAULT, INPUT_PULLUP);
  sharedSpiUseEncoder();

  const bool currentAdcOk = currentAdc.init(ADC_SCK_HZ);
  board_pwm::start();

  currentSense0.linkDriver(&driver0);
  currentSense1.linkDriver(&driver1);
  currentSense0.skip_align = true;
  currentSense1.skip_align = true;
  const bool currentSense0Ok = currentAdcOk && driver0Ok && currentSense0.init();
  const bool currentSense1Ok = currentAdcOk && driver1Ok && currentSense1.init();

  status.currentFeedback0Ok = currentAdcOk && currentSense0Ok;
  status.currentFeedback1Ok = currentAdcOk && currentSense1Ok;
  status.encoder0Allowed =
    encoder0.angleOk() && (!REQUIRE_ENCODER_STARTUP_HEALTH || encoder0Healthy);
  status.encoder1Allowed =
    encoder1.angleOk() && (!REQUIRE_ENCODER_STARTUP_HEALTH || encoder1Healthy);

  motorHardwareInitialized = true;
  return status;
}



void controlSetup() {
  normalControlMode = true;
  const MotorHardwareStatus hardware = initializeMotorHardware();
  initializeRuntimeBusVoltage(hardware.busVoltage, micros());

  if (hardware.encoder0Allowed && hardware.currentFeedback0Ok &&
      control_math::electricalCalibrationValid(runtimeConfig0.sensorDirectionSign, runtimeConfig0.zeroElectricAngle)) {
    if (configureMotor(motor0, driver0, currentSense0, encoder0, runtimeConfig0)) {
      motor0Ready = startClosedLoopMotor(motor0, control0);
    } else {
      driver0.disable();
    }
  } else {
    driver0.disable();
  }

  if (hardware.encoder1Allowed && hardware.currentFeedback1Ok &&
      control_math::electricalCalibrationValid(runtimeConfig1.sensorDirectionSign, runtimeConfig1.zeroElectricAngle)) {
    if (configureMotor(motor1, driver1, currentSense1, encoder1, runtimeConfig1)) {
      motor1Ready = startClosedLoopMotor(motor1, control1);
    } else {
      driver1.disable();
    }
  } else {
    driver1.disable();
  }

  if (motor0Ready) encoder0.setStartupReference(runtimeConfig0.sensorOffset, runtimeConfig0.sensorDirectionSign);
  if (motor1Ready) encoder1.setStartupReference(runtimeConfig1.sensorOffset, runtimeConfig1.sensorDirectionSign);
  settleStartupTargets(STARTUP_TARGET_SETTLE_MS);
  serviceEmergencyStop();
  publishRuntimeState();
  current_feedback::refresh(); // discard the last setup frame
  lastControlFrame = current_feedback::sequence();
}

static void stopAllMotors() {
  commandTorqueEnabled = commandMotor0Enabled = commandMotor1Enabled = false;
  control0.iqCommand = control1.iqCommand = 0.0f;
  setIqTarget(motor0, 0.0f);
  setIqTarget(motor1, 0.0f);
  current_feedback::freeze(false);
  board_pwm::stop();
  if (motor0.driver) motor0.disable();
  if (motor1.driver) motor1.disable();
  if (driver0.initialized) driver0.setDriverOffEnabled(true);
  if (driver1.initialized) driver1.setDriverOffEnabled(true);
  motor0Ready = motor1Ready = false;
}

static void latchControlFault(ControlFault cause) {
  if (runtimeFault) return;
  if (cause == ControlFault::EmergencyStop)
    readyBeforeEmergencyStop = (motor0Ready ? 1u : 0u) | (motor1Ready ? 2u : 0u);
  runtimeFault = true;
  runtimeFaultCause = cause;
  stopAllMotors();
}

static bool recoverEmergencyStop(uint32_t pressGeneration) {
  // Keep the GPIO inhibit asserted throughout validation and driver enabling.
  // No alignment or offset calibration is run during recovery.
  auto fail = [](ControlFault cause) {
    runtimeFaultCause = cause;
    stopAllMotors();
    return false;
  };
  for (auto *driver : {&driver0, &driver1}) {
    if (!driver->initialized) continue;
    const auto status = driver->getStatus();
    if (status.status.FAULT || status.status.SPI_FLT) return fail(ControlFault::Driver);
    driver->setDriverOffEnabled(false);
  }
  if (!gpio_get(GPIO_DRV_Mx_nFAULT)) return fail(ControlFault::Driver);
  current_feedback::refresh();
  if (!current_feedback::waitNext(CURRENT_FEEDBACK_TIMEOUT_US))
    return fail(ControlFault::CurrentFeedback);

  auto prepare = [](BLDCMotor &motor, CheckedAS5048ASensor &encoder, PositionHoldState &control) {
    encoder.update(); // discard the pipelined response from before driver SPI
    encoder.update();
    if (!encoder.angleOk() || !encoder.feedbackFresh(micros())) return false;
    encoder.resetVelocity();
    control = {};
    motor.current = {};
    motor.voltage = {};
    setIqTarget(motor, 0.0f);
    motor.enable(); // resets the current and motion integrators
    updateMotorKinematics(motor);
    control.targetPosition = motor.shaft_angle;
    return true;
  };
  if ((readyBeforeEmergencyStop & 1u) && !prepare(motor0, encoder0, control0))
    return fail(ControlFault::Encoder0);
  if ((readyBeforeEmergencyStop & 2u) && !prepare(motor1, encoder1, control1))
    return fail(ControlFault::Encoder1);

  board_pwm::stop();
  if (!emergency_stop::releaseInhibit(pressGeneration)) return fail(ControlFault::EmergencyStop);
  discardControlReferenceCommands(lastAppliedCommandSequence);
  commandTorqueEnabled = commandMotor0Enabled = commandMotor1Enabled = false;
  commandTimeoutUs = 0;
  runtimeFault = false;
  runtimeFaultCause = ControlFault::None;
  motor0Ready = (readyBeforeEmergencyStop & 1u) != 0;
  motor1Ready = (readyBeforeEmergencyStop & 2u) != 0;
  lastControlLoopUs = 0;
  lastRuntimePublishUs = micros();
  lastRuntimePublishLoopCounter = controlLoopCounter;
  if (normalControlMode) publishRuntimeState();
  current_feedback::refresh();
  lastControlFrame = current_feedback::sequence();
  return true;
}

void serviceEmergencyStop() {
  emergency_stop::poll();
  if (!emergency_stop::active() || !motorHardwareInitialized) return;
  if (!runtimeFault) {
    latchControlFault(ControlFault::EmergencyStop);
    if (normalControlMode) publishRuntimeState();
    else Serial.println("E-STOP: motors disabled. Release, then hold again: reset occurs at 3 s.");
  }
  if (runtimeFaultCause != ControlFault::EmergencyStop) return;
  uint32_t generation;
  if (emergency_stop::resetRequested(generation)) {
    const bool ok = recoverEmergencyStop(generation);
    if (!normalControlMode)
      Serial.println(ok ? "E-stop cleared; start a new operation." : "Recovery failed; motors remain disabled.");
  }
}

static bool controlMotor(BLDCMotor &motor, CheckedAS5048ASensor &encoder,
                         PositionHoldState &control, const PositionHoldConfig &config,
                         bool ready, bool torqueEnabled) {
  if (!ready) return true;
  encoder.prepareControlSample();
  updateMotorKinematics(motor);
  setIqTarget(motor, torqueEnabled ? runPositionHoldPd(motor, control, config) : 0.0f);
  if (!torqueEnabled) control.iqCommand = 0.0f;
  motor.loopFOC();
  return encoder.feedbackFresh(micros()) && isfinite(motor.current.q) &&
    isfinite(motor.current.d) && isfinite(motor.voltage.q) && isfinite(motor.voltage.d);
}

void controlStep() {
  serviceEmergencyStop();
  const bool newFrame = current_feedback::refresh();
  const uint32_t nowUs = micros();
  if (!motor0Ready && !motor1Ready) applyControlReferenceCommandIfAvailable();
  if (newFrame && runtimeFaultCause == ControlFault::EmergencyStop) {
    // Preserve turn tracking if the shafts are moved while stopped.
    if (readyBeforeEmergencyStop & 1u) encoder0.update();
    if (readyBeforeEmergencyStop & 2u) encoder1.update();
  }
  if (motor0Ready || motor1Ready) {
    if (!gpio_get(GPIO_DRV_Mx_nFAULT)) latchControlFault(ControlFault::Driver);
    else if (!current_feedback::healthy(nowUs)) latchControlFault(ControlFault::CurrentFeedback);
  }

  if (newFrame && !runtimeFault && (motor0Ready || motor1Ready)) {
    if (current_feedback::sequence() - lastControlFrame != 1u) {
      latchControlFault(ControlFault::AdcFrameGap);
    } else if (!board_pwm::samplingWindow()) {
      latchControlFault(ControlFault::SampleTiming);
    } else if (!board_pwm::commitStagedFrame()) {
      latchControlFault(ControlFault::PwmDeadline);
    } else {
      lastControlFrame = current_feedback::sequence();
      // Apply the previous frame first, then compute this one. Both FOC calls
      // have a full PWM period; all phases still latch together at the next zero.
      board_pwm::beginFrame();
      current_feedback::freeze(true);
      applyControlReferenceCommandIfAvailable();
      const bool torqueAllowed = commandTorqueAllowed(micros());
      const bool ok0 = controlMotor(motor0, encoder0, control0, runtimeConfig0,
                                   motor0Ready, torqueAllowed && commandMotor0Enabled);
      const bool ok1 = controlMotor(motor1, encoder1, control1, runtimeConfig1,
                                   motor1Ready, torqueAllowed && commandMotor1Enabled);
      current_feedback::freeze(false);
      if (emergency_stop::active()) serviceEmergencyStop();
      else if (motor0Ready && !encoder0.feedbackFresh(micros())) latchControlFault(ControlFault::Encoder0);
      else if (motor1Ready && !encoder1.feedbackFresh(micros())) latchControlFault(ControlFault::Encoder1);
      else if (!ok0 || !ok1) latchControlFault(ControlFault::NonFiniteControl);
      else if (!gpio_get(GPIO_DRV_Mx_nFAULT)) latchControlFault(ControlFault::Driver);
      else if (!board_pwm::finishFrame()) latchControlFault(ControlFault::PwmDeadline);
      controlLoopCounter++;
    }
  }

  updateRuntimeBusVoltageIfDue(nowUs);
  // Telemetry stays alive after a latched fault or when calibration is absent.
  if ((nowUs - lastRuntimePublishUs) >= RUNTIME_STATE_PUBLISH_INTERVAL_US) {
    const uint32_t loopDelta = controlLoopCounter - lastRuntimePublishLoopCounter;
    const uint32_t elapsedUs = lastRuntimePublishUs == 0 ? 0 : nowUs - lastRuntimePublishUs;
    lastControlLoopUs = loopDelta && elapsedUs ? float(elapsedUs) / float(loopDelta) : 0.0f;
    lastRuntimePublishLoopCounter = controlLoopCounter;
    lastRuntimePublishUs = nowUs;
    publishRuntimeState();
  }
}
