// Dual DRV8316C + GM3506 current-FOC position hold.
// SimpleFOC handles FOC commutation; this file supplies a small PD loop that
// commands Iq in closed-loop current control.

#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/spi.h"
#include "board_config.h"
#include "BU79100QuadReader.h"
#include "drivers/drv8316/drv8316.h"

static PhaseCurrent_s readMotor0Currents();
static PhaseCurrent_s readMotor1Currents();

bool core1_disable_systick = true;
bool core1_separate_stack = true;

static constexpr uint8_t CONTROL_REFERENCE_FLAG_M0 = 1u << 0;
static constexpr uint8_t CONTROL_REFERENCE_FLAG_M1 = 1u << 1;

struct PositionHoldState {
  float targetPosition = 0.0f;
  float targetVelocity = 0.0f;
  float feedforwardCurrent = 0.0f;
  float iqCommand = 0.0f;
};

struct EncoderRegisterDiagnostics {
  uint16_t magnitude = 0;
  uint16_t diagnostics = 0;
  bool magnitudeOk = false;
  bool diagnosticsOk = false;
};

struct RuntimeMotorState {
  float position = 0.0f;
  float velocity = 0.0f;
  float iq = 0.0f;
  float iqTarget = 0.0f;
  bool ready = false;
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

struct __attribute__((packed)) UsbStatePacket {
  uint8_t magic0;
  uint8_t magic1;
  uint8_t type;
  uint8_t version;
  uint8_t length;
  uint16_t sequence;
  uint32_t t_us;
  uint32_t latest_command_index;
  float control_loop_us;
  float m0_q;
  float m0_v;
  float m0_i;
  float m0_i_target;
  float m1_q;
  float m1_v;
  float m1_i;
  float m1_i_target;
  uint8_t flags;
  uint8_t checksum;
};

struct __attribute__((packed)) UsbCommandPacket {
  uint8_t magic0;
  uint8_t magic1;
  uint8_t type;
  uint8_t version;
  uint8_t length;
  uint32_t command_index;
  uint8_t flags;
  uint16_t timeout_ms;
  float m0_kp;
  float m0_kd;
  float m0_iff;
  float m0_q_target;
  float m0_v_target;
  float m1_kp;
  float m1_kd;
  float m1_iff;
  float m1_q_target;
  float m1_v_target;
  uint8_t checksum;
};

static_assert(sizeof(UsbStatePacket) == 53, "Unexpected USB state packet size");
static_assert(sizeof(UsbCommandPacket) == 53, "Unexpected USB command packet size");

class CheckedAS5048ASensor : public Sensor {
public:
  explicit CheckedAS5048ASensor(uint8_t csPin)
    : csPin_(csPin), settings_(ENCODER_SPI_HZ, MSBFIRST, SPI_MODE1) {}

  void init(SPIClass *spi = &SPI, spi_inst_t *spiHw = nullptr) {
    spi_ = spi;
    spiHw_ = spiHw;
    angleOk_ = false;
    pinMode(csPin_, OUTPUT);
    digitalWrite(csPin_, HIGH);
    if (spiHw_ != nullptr) {
      configureHardwareSpi();
      fastRuntimeSpi_ = true;
    } else {
      spi_->begin();
      fastRuntimeSpi_ = false;
    }

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
        return;
      }
      delayMicroseconds(50);
    }

    angle_prev = 0.0f;
    vel_angle_prev = 0.0f;
    angle_prev_ts = micros();
    vel_angle_prev_ts = angle_prev_ts;
  }

  void enableFastRuntimeSpi(spi_inst_t *spiHw) {
    if (spiHw == nullptr) {
      fastRuntimeSpi_ = false;
      return;
    }

    spiHw_ = spiHw;
    configureHardwareSpi();
    fastRuntimeSpi_ = true;
  }

  float getSensorAngle() override {
    uint16_t raw = 0;
    if (!readRawChecked(raw)) {
      return -1.0f;
    }

    return rawToAngle(raw);
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

  void configureHardwareSpi() {
    if (spiHw_ != nullptr) {
      spi_init(spiHw_, ENCODER_SPI_HZ);
      spi_set_baudrate(spiHw_, ENCODER_SPI_HZ);
      spi_set_format(spiHw_, 16, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
    }
  }

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

  static uint16_t makeReadCommand(uint16_t reg) {
    uint16_t command = (reg & AS5048A_RESULT_MASK) | AS5048A_READ_BIT;
    if (!hasEvenParity(command)) {
      command |= AS5048A_PARITY_BIT;
    }
    return command;
  }

  uint16_t transfer16(uint16_t out) {
    if (fastRuntimeSpi_ && spiHw_ != nullptr) {
      uint16_t in = 0;
      gpio_put(csPin_, false);
      spi_write16_read16_blocking(spiHw_, &out, &in, 1);
      gpio_put(csPin_, true);
      return in;
    }

    spi_->beginTransaction(settings_);
    digitalWrite(csPin_, LOW);
    const uint16_t in = spi_->transfer16(out);
    digitalWrite(csPin_, HIGH);
    spi_->endTransaction();
    delayMicroseconds(1);
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
  SPIClass *spi_ = &SPI;
  spi_inst_t *spiHw_ = nullptr;
  SPISettings settings_;
  bool fastRuntimeSpi_ = false;
  bool angleOk_ = false;
};

class FastCallbackCurrentSense : public CurrentSense {
public:
  explicit FastCallbackCurrentSense(PhaseCurrent_s (*readCallback)())
    : readCallback_(readCallback) {}

  int init() override {
    if (readCallback_ == nullptr) {
      initialized = false;
      return 0;
    }

    offset_ia = 0.0f;
    offset_ib = 0.0f;
    offset_ic = 0.0f;

    for (uint16_t i = 0; i < CURRENT_SENSE_CALIBRATION_SAMPLES; i++) {
      const PhaseCurrent_s current = readCallback_();
      offset_ia += current.a;
      offset_ib += current.b;
      offset_ic += current.c;
      delayMicroseconds(CURRENT_SENSE_CALIBRATION_SAMPLE_US);
    }

    const float invSamples = 1.0f / (float)CURRENT_SENSE_CALIBRATION_SAMPLES;
    offset_ia *= invSamples;
    offset_ib *= invSamples;
    offset_ic *= invSamples;
    initialized = true;
    return 1;
  }

  PhaseCurrent_s getPhaseCurrents() override {
    PhaseCurrent_s current = readCallback_();
    current.a -= offset_ia;
    current.b -= offset_ib;
    current.c -= offset_ic;
    lastCurrent_ = current;
    return current;
  }

  PhaseCurrent_s lastPhaseCurrents() const {
    return lastCurrent_;
  }

  int driverAlign(float align_voltage, bool modulation_centered = false) override {
    _UNUSED(align_voltage);
    _UNUSED(modulation_centered);
    return initialized ? 1 : 0;
  }

private:
  PhaseCurrent_s (*readCallback_)() = nullptr;
  PhaseCurrent_s lastCurrent_ = {0.0f, 0.0f, 0.0f};
};

DRV8316Driver3PWM driver0(
  GPIO_M0_PWM_A,
  GPIO_M0_PWM_B,
  GPIO_M0_PWM_C,
  GPIO_M0_DRV_CS,
  false,
  NOT_SET,
  GPIO_DRV_Mx_nFAULT
);

DRV8316Driver3PWM driver1(
  GPIO_M1_PWM_A,
  GPIO_M1_PWM_B,
  GPIO_M1_PWM_C,
  GPIO_M1_DRV_CS,
  false,
  NOT_SET,
  GPIO_DRV_Mx_nFAULT
);

BLDCMotor motor0(GM3506_POLE_PAIRS);
BLDCMotor motor1(GM3506_POLE_PAIRS);
CheckedAS5048ASensor encoder0(GPIO_M0_ENC_CS);
CheckedAS5048ASensor encoder1(GPIO_M1_ENC_CS);
BU79100QuadReader currentAdc(pio0, GPIO_ADC_SCK, GPIO_ADC_CSB, GPIO_M0_ADC_DATA_A, GPIO_ADC_SYNC_PWM);
FastCallbackCurrentSense currentSense0(readMotor0Currents);
FastCallbackCurrentSense currentSense1(readMotor1Currents);

static uint32_t lastRuntimePublishUs = 0;
static uint16_t telemetrySequence = 0;
static PositionHoldState control0;
static PositionHoldState control1;
static PositionHoldConfig runtimeConfig0 = MOTOR0_CONFIG;
static PositionHoldConfig runtimeConfig1 = MOTOR1_CONFIG;
static bool motor0Ready = false;
static bool motor1Ready = false;
static bool currentFeedback0Ready = false;
static bool currentFeedback1Ready = false;
static uint32_t controlLoopCounter = 0;
static uint32_t lastRuntimePublishLoopCounter = 0;
static uint32_t latestAppliedCommandIndex = 0;
static uint32_t lastCommandApplyUs = 0;
static uint32_t commandTimeoutUs = 0;
static float lastControlLoopUs = 0.0f;
static bool commandTorqueEnabled = false;
static bool commandMotor0Enabled = false;
static bool commandMotor1Enabled = false;

static volatile bool interfaceCoreReady = false;
// Latest-value mailboxes use an odd/even sequence counter so readers never see torn structs.
static volatile uint32_t runtimeStateSequence = 0;
static RuntimeControlState sharedRuntimeState;
static volatile uint32_t controlReferenceSequence = 0;
static ControlReferenceCommand sharedControlReference;

static void sharedMemoryBarrier() {
  __asm__ volatile("dmb sy" ::: "memory");
}

static void publishRuntimeControlState(const RuntimeControlState &state) {
  uint32_t sequence = runtimeStateSequence;
  if ((sequence & 1u) != 0) {
    sequence++;
  }

  runtimeStateSequence = sequence + 1u;
  sharedMemoryBarrier();
  sharedRuntimeState = state;
  sharedMemoryBarrier();
  runtimeStateSequence = sequence + 2u;
}

static bool readLatestRuntimeControlState(RuntimeControlState &state) {
  uint32_t sequenceBefore;
  uint32_t sequenceAfter;

  do {
    sequenceBefore = runtimeStateSequence;
    if (sequenceBefore == 0 || (sequenceBefore & 1u) != 0) {
      return false;
    }
    sharedMemoryBarrier();
    state = sharedRuntimeState;
    sharedMemoryBarrier();
    sequenceAfter = runtimeStateSequence;
  } while (sequenceBefore != sequenceAfter || (sequenceAfter & 1u) != 0);

  return true;
}

static void publishControlReferenceCommand(const ControlReferenceCommand &command) {
  uint32_t sequence = controlReferenceSequence;
  if ((sequence & 1u) != 0) {
    sequence++;
  }

  controlReferenceSequence = sequence + 1u;
  sharedMemoryBarrier();
  sharedControlReference = command;
  sharedMemoryBarrier();
  controlReferenceSequence = sequence + 2u;
}

static bool readLatestControlReferenceCommand(
  ControlReferenceCommand &command,
  uint32_t &sequence
) {
  uint32_t sequenceBefore;
  uint32_t sequenceAfter;

  do {
    sequenceBefore = controlReferenceSequence;
    if (sequenceBefore == 0 || (sequenceBefore & 1u) != 0) {
      return false;
    }
    sharedMemoryBarrier();
    command = sharedControlReference;
    sharedMemoryBarrier();
    sequenceAfter = controlReferenceSequence;
  } while (sequenceBefore != sequenceAfter || (sequenceAfter & 1u) != 0);

  sequence = sequenceAfter;
  return true;
}

static void deselectSpiSlaves() {
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
}

static void configureAdcTriggerPwm() {
  gpio_set_function(GPIO_ADC_SYNC_PWM, GPIO_FUNC_PWM);

  const uint slice = pwm_gpio_to_slice_num(GPIO_ADC_SYNC_PWM);
  const uint channel = pwm_gpio_to_channel(GPIO_ADC_SYNC_PWM);
  const uint32_t sysclockHz = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS) * 1000;
  const uint32_t dividerFactor = 4096 * 2 * PWM_FREQUENCY;
  uint32_t divider = sysclockHz / dividerFactor;
  if ((sysclockHz % dividerFactor) != 0) divider += 1;
  if (divider < 16) divider = 16;
  const uint32_t wrap = (sysclockHz * 8) / divider / PWM_FREQUENCY - 1;

  pwm_config config = pwm_get_default_config();
  pwm_config_set_clkdiv_int_frac(&config, divider >> 4, divider & 0xF);
  pwm_config_set_phase_correct(&config, true);
  pwm_config_set_wrap(&config, wrap);
  pwm_init(slice, &config, false);
  pwm_set_chan_level(slice, channel, (uint16_t)((wrap + 1) * ADC_TRIGGER_DUTY));
}

static void syncAllPwmSlices() {
  for (uint i = 0; i < NUM_PWM_SLICES; i++) {
    pwm_set_enabled(i, false);
    pwm_set_counter(i, 0);
  }
  pwm_set_mask_enabled((1u << NUM_PWM_SLICES) - 1u);
}

static float readBusVoltage() {
  uint32_t sum = 0;

  analogReadResolution(VBUS_ADC_BITS);
  pinMode(GPIO_VBUS_SENSE, INPUT);
  delay(2);

  for (uint16_t i = 0; i < VBUS_STARTUP_SAMPLES; i++) {
    sum += analogRead(GPIO_VBUS_SENSE);
    delayMicroseconds(50);
  }

  const float rawCounts = (float)sum / VBUS_STARTUP_SAMPLES;
  const float adcVoltage = rawCounts * CURRENT_SENSE_VREF / VBUS_ADC_MAX_COUNTS;
  return adcVoltage * VBUS_DIVIDER_RATIO;
}

static Direction directionFromSign(int8_t sign) {
  if (sign > 0) {
    return Direction::CW;
  }
  if (sign < 0) {
    return Direction::CCW;
  }
  return Direction::UNKNOWN;
}

static float usableSupplyVoltage(float measuredVoltage) {
  if (measuredVoltage > 4.0f && measuredVoltage < 30.0f) {
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

static void configureDriver(DRV8316Driver3PWM &driver, float measuredBusVoltage) {
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

  driver.setCurrentSenseGain(DRV8316_CSAGain::Gain_0V375);
  delayMicroseconds(5);

  driver.setDriverOffEnabled(false);
  delayMicroseconds(5);

  driver.clearFault();
}

static float adcRawToSensedCurrent(uint16_t raw) {
  const float centeredCounts = (float)raw - ADC_ZERO_CURRENT_COUNTS;
  return centeredCounts * ADC_COUNT_TO_PHASE_CURRENT_A;
}

static PhaseCurrent_s decoupleDRV8316Currents(uint16_t rawA, uint16_t rawC) {
  const float iaSensed = adcRawToSensedCurrent(rawA);
  const float icSensed = adcRawToSensedCurrent(rawC);

  // TI DRV8316 datasheet calibration for current sensing on phases A and C.
  const float ia = 1.025489f * iaSensed + 0.011936f * icSensed;
  const float ic = 0.022043f * iaSensed + 0.996548f * icSensed;

  return {
    ia,
    -ia - ic,
    ic,
  };
}

static PhaseCurrent_s readMotor0Currents() {
  const BU79100QuadSample sample = currentAdc.read();
  return decoupleDRV8316Currents(sample.raw[0], sample.raw[1]);
}

static PhaseCurrent_s readMotor1Currents() {
  const BU79100QuadSample sample = currentAdc.read();
  return decoupleDRV8316Currents(sample.raw[2], sample.raw[3]);
}

static bool encoderOcf(uint16_t diagnostics) {
  return ((diagnostics >> 8) & 0x1) != 0;
}

static bool encoderCof(uint16_t diagnostics) {
  return ((diagnostics >> 9) & 0x1) != 0;
}

static bool encoderCompLow(uint16_t diagnostics) {
  return ((diagnostics >> 10) & 0x1) != 0;
}

static bool encoderCompHigh(uint16_t diagnostics) {
  return ((diagnostics >> 11) & 0x1) != 0;
}

static bool encoderDiagnosticsHealthy(const EncoderRegisterDiagnostics &regs) {
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

static bool configureMotor(
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
  motor.foc_modulation = FOCModulationType::SinePWM;
  const float voltageLimit = _constrain(CURRENT_FOC_VOLTAGE_LIMIT, 0.0f, driver.voltage_limit);
  motor.phase_resistance = NOT_SET;
  motor.phase_inductance = GM3506_PHASE_INDUCTANCE_H;
  motor.sensor_direction = directionFromSign(config.sensorDirectionSign);
  motor.zero_electric_angle = config.zeroElectricAngle;
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

static void setIqTarget(BLDCMotor &motor, float iq) {
  motor.target = iq;
  motor.current_sp = iq;
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
  static uint32_t lastAppliedSequence = 0;
  const uint32_t observedSequence = controlReferenceSequence;

  if (observedSequence == 0 ||
      (observedSequence & 1u) != 0 ||
      observedSequence == lastAppliedSequence) {
    return;
  }

  ControlReferenceCommand command;
  uint32_t sequence;

  if (!readLatestControlReferenceCommand(command, sequence)) {
    return;
  }
  lastAppliedSequence = sequence;
  latestAppliedCommandIndex = command.commandIndex;
  lastCommandApplyUs = micros();
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
  const float angle = motor.shaftAngle();
  const float velocity = motor.shaftVelocity();
  const float angleError = control.targetPosition - angle;
  const float velocityError = control.targetVelocity - velocity;
  const float iq = config.kp * angleError +
    config.kd * velocityError +
    control.feedforwardCurrent;

  motor.shaft_angle = angle;
  motor.shaft_velocity = velocity;
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
      motor0.loopFOC();
      setIqTarget(motor0, 0.0f);
    }
    if (motor1Ready) {
      motor1.loopFOC();
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

static uint8_t packetChecksum(const uint8_t *bytes, size_t length) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < (length - 1); i++) {
    checksum ^= bytes[i];
  }
  return checksum;
}

static uint8_t statePacketChecksum(const UsbStatePacket &packet) {
  const uint8_t *bytes = (const uint8_t *)&packet;
  return packetChecksum(bytes, sizeof(UsbStatePacket));
}

static bool commandPacketChecksumOk(const UsbCommandPacket &packet) {
  const uint8_t *bytes = (const uint8_t *)&packet;
  return packetChecksum(bytes, sizeof(UsbCommandPacket)) == packet.checksum;
}

static UsbStatePacket makeUsbStatePacket(const RuntimeControlState &state) {
  UsbStatePacket packet = {};
  packet.magic0 = USB_PACKET_MAGIC0;
  packet.magic1 = USB_PACKET_MAGIC1;
  packet.type = USB_PACKET_TYPE_STATE;
  packet.version = USB_PACKET_VERSION;
  packet.length = sizeof(UsbStatePacket);
  packet.sequence = telemetrySequence++;
  packet.t_us = state.tUs;
  packet.latest_command_index = state.latestCommandIndex;
  packet.control_loop_us = state.controlLoopUs;

  packet.m0_q = state.m0.position;
  packet.m0_v = state.m0.velocity;
  packet.m0_i = state.m0.iq;
  packet.m0_i_target = state.m0.iqTarget;

  packet.m1_q = state.m1.position;
  packet.m1_v = state.m1.velocity;
  packet.m1_i = state.m1.iq;
  packet.m1_i_target = state.m1.iqTarget;

  packet.flags = state.flags;
  packet.checksum = statePacketChecksum(packet);

  return packet;
}

static RuntimeMotorState makeRuntimeMotorState(
  BLDCMotor &motor,
  const PositionHoldState &control,
  bool ready
) {
  RuntimeMotorState state;
  state.position = ready ? motor.shaft_angle : 0.0f;
  state.velocity = ready ? motor.shaft_velocity : 0.0f;
  state.iq = ready ? motor.current.q : 0.0f;
  state.iqTarget = ready ? control.iqCommand : 0.0f;
  state.ready = ready;
  return state;
}

static RuntimeControlState makeRuntimeControlState() {
  RuntimeControlState state;
  state.tUs = micros();
  state.latestCommandIndex = latestAppliedCommandIndex;
  state.controlLoopUs = lastControlLoopUs;
  state.flags = (motor0Ready ? USB_STATE_FLAG_M0_READY : 0) |
    (motor1Ready ? USB_STATE_FLAG_M1_READY : 0);
  state.m0 = makeRuntimeMotorState(motor0, control0, motor0Ready);
  state.m1 = makeRuntimeMotorState(motor1, control1, motor1Ready);
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

static bool tryStartMotor(
  BLDCMotor &motor,
  DRV8316Driver3PWM &driver,
  CurrentSense &currentSense,
  CheckedAS5048ASensor &encoder,
  PositionHoldState &control,
  PositionHoldConfig &config,
  bool currentFeedbackReady,
  bool &motorReady
) {
  if (motorReady || !currentFeedbackReady) {
    return motorReady;
  }

  encoder.init(&SPI, spi0);
  EncoderRegisterDiagnostics regs;
  const bool encoderHealthy = checkStartupEncoderHealth(encoder, regs);

  const bool encoderAllowed =
    encoder.angleOk() && (!REQUIRE_ENCODER_STARTUP_HEALTH || encoderHealthy);
  if (!encoderAllowed) {
    driver.disable();
    return false;
  }

  if (configureMotor(motor, driver, currentSense, encoder, config)) {
    motorReady = startClosedLoopMotor(motor, control);
  } else {
    motorReady = false;
  }

  if (!motorReady) {
    driver.disable();
    return false;
  }

  publishRuntimeState();
  return true;
}

static void retrySkippedMotorsIfDue() {
  static uint32_t lastRetryMs = 0;
  if (motor0Ready && motor1Ready) {
    return;
  }

  const uint32_t nowMs = millis();
  if ((nowMs - lastRetryMs) < MOTOR_START_RETRY_INTERVAL_MS) {
    return;
  }
  lastRetryMs = nowMs;

  if (!motor0Ready) {
    tryStartMotor(
      motor0,
      driver0,
      currentSense0,
      encoder0,
      control0,
      runtimeConfig0,
      currentFeedback0Ready,
      motor0Ready
    );
  }
  if (!motor1Ready) {
    tryStartMotor(
      motor1,
      driver1,
      currentSense1,
      encoder1,
      control1,
      runtimeConfig1,
      currentFeedback1Ready,
      motor1Ready
    );
  }

  publishRuntimeState();
}

static void writeUsbStatePacketIfDue(const RuntimeControlState &state) {
  static uint32_t lastStatePacketUs = 0;

  if (!Serial) {
    return;
  }
  if (state.tUs == lastStatePacketUs) {
    return;
  }
  if (lastStatePacketUs != 0 &&
      (state.tUs - lastStatePacketUs) < USB_STATE_FRAME_INTERVAL_US) {
    return;
  }
  if (Serial.availableForWrite() < (int)sizeof(UsbStatePacket)) {
    lastStatePacketUs = state.tUs;
    return;
  }

  const UsbStatePacket packet = makeUsbStatePacket(state);
  Serial.write((const uint8_t *)&packet, sizeof(packet));
  lastStatePacketUs = state.tUs;
}

static bool commandPacketHeaderOk(const UsbCommandPacket &packet) {
  return packet.magic0 == USB_PACKET_MAGIC0 &&
    packet.magic1 == USB_PACKET_MAGIC1 &&
    packet.type == USB_PACKET_TYPE_COMMAND &&
    packet.version == USB_PACKET_VERSION &&
    packet.length == sizeof(UsbCommandPacket);
}

static MotorReferenceCommand makeMotorReferenceCommand(
  float kp,
  float kd,
  float iff,
  float qTarget,
  float vTarget
) {
  MotorReferenceCommand command;
  command.kp = kp;
  command.kd = kd;
  command.feedforwardCurrent = iff;
  command.targetPosition = qTarget;
  command.targetVelocity = vTarget;
  return command;
}

static void publishUsbCommandPacket(const UsbCommandPacket &packet) {
  ControlReferenceCommand command;
  command.tUs = micros();
  command.commandIndex = packet.command_index;
  command.timeoutMs = packet.timeout_ms;
  command.flags = packet.flags & (CONTROL_REFERENCE_FLAG_M0 | CONTROL_REFERENCE_FLAG_M1);
  command.m0 = makeMotorReferenceCommand(
    packet.m0_kp,
    packet.m0_kd,
    packet.m0_iff,
    packet.m0_q_target,
    packet.m0_v_target
  );
  command.m1 = makeMotorReferenceCommand(
    packet.m1_kp,
    packet.m1_kd,
    packet.m1_iff,
    packet.m1_q_target,
    packet.m1_v_target
  );

  publishControlReferenceCommand(command);
}

static void readUsbCommandPackets() {
  static uint8_t rx[sizeof(UsbCommandPacket)];
  static uint8_t rxCount = 0;

  while (Serial.available() > 0) {
    const int value = Serial.read();
    if (value < 0) {
      return;
    }

    const uint8_t byte = (uint8_t)value;
    if (rxCount == 0) {
      if (byte == USB_PACKET_MAGIC0) {
        rx[rxCount++] = byte;
      }
      continue;
    }

    if (rxCount == 1) {
      if (byte == USB_PACKET_MAGIC1) {
        rx[rxCount++] = byte;
      } else if (byte != USB_PACKET_MAGIC0) {
        rxCount = 0;
      }
      continue;
    }

    rx[rxCount++] = byte;

    if (rxCount == 5 && rx[4] != sizeof(UsbCommandPacket)) {
      rxCount = (byte == USB_PACKET_MAGIC0) ? 1 : 0;
      if (rxCount == 1) {
        rx[0] = byte;
      }
      continue;
    }

    if (rxCount < sizeof(UsbCommandPacket)) {
      continue;
    }

    UsbCommandPacket packet;
    memcpy(&packet, rx, sizeof(packet));
    rxCount = 0;

    if (commandPacketHeaderOk(packet) && commandPacketChecksumOk(packet)) {
      publishUsbCommandPacket(packet);
    }
  }
}

static void controlSetup() {
  deselectSpiSlaves();
  pinMode(GPIO_DRV_Mx_nFAULT, INPUT_PULLUP);
  configureSpiPins();

  const float busVoltage = readBusVoltage();

  delay(ENCODER_POWERUP_DELAY_MS);
  encoder0.init(&SPI, spi0);
  encoder1.init(&SPI, spi0);
  EncoderRegisterDiagnostics encoder0Regs;
  EncoderRegisterDiagnostics encoder1Regs;
  const bool encoder0Healthy = checkStartupEncoderHealth(encoder0, encoder0Regs);
  const bool encoder1Healthy = checkStartupEncoderHealth(encoder1, encoder1Regs);

  configureDriver(driver0, busVoltage);
  configureDriver(driver1, busVoltage);
  encoder0.enableFastRuntimeSpi(spi0);
  encoder1.enableFastRuntimeSpi(spi0);

  configureAdcTriggerPwm();
  syncAllPwmSlices();
  const bool currentAdcOk = currentAdc.init(ADC_SCK_HZ);

  currentSense0.linkDriver(&driver0);
  currentSense1.linkDriver(&driver1);
  currentSense0.skip_align = true;
  currentSense1.skip_align = true;
  const bool currentSense0Ok = currentSense0.init();
  const bool currentSense1Ok = currentSense1.init();
  const bool currentFeedback0Ok = currentAdcOk && currentSense0Ok;
  const bool currentFeedback1Ok = currentAdcOk && currentSense1Ok;
  currentFeedback0Ready = currentFeedback0Ok;
  currentFeedback1Ready = currentFeedback1Ok;

  const bool encoder0Allowed =
    encoder0.angleOk() && (!REQUIRE_ENCODER_STARTUP_HEALTH || encoder0Healthy);
  const bool encoder1Allowed =
    encoder1.angleOk() && (!REQUIRE_ENCODER_STARTUP_HEALTH || encoder1Healthy);

  if (encoder0Allowed && currentFeedback0Ok) {
    if (configureMotor(motor0, driver0, currentSense0, encoder0, runtimeConfig0)) {
      motor0Ready = startClosedLoopMotor(motor0, control0);
    } else {
      driver0.disable();
    }
  } else {
    driver0.disable();
  }

  if (encoder1Allowed && currentFeedback1Ok) {
    if (configureMotor(motor1, driver1, currentSense1, encoder1, runtimeConfig1)) {
      motor1Ready = startClosedLoopMotor(motor1, control1);
    } else {
      driver1.disable();
    }
  } else {
    driver1.disable();
  }

  settleStartupTargets(STARTUP_TARGET_SETTLE_MS);
  publishRuntimeState();
}

static void controlStep() {
  applyControlReferenceCommandIfAvailable();
  const uint32_t nowUs = micros();
  const bool torqueAllowed = commandTorqueAllowed(nowUs);

  if (motor0Ready) {
    motor0.loopFOC();
    if (torqueAllowed && commandMotor0Enabled) {
      const float iq = runPositionHoldPd(motor0, control0, runtimeConfig0);
      setIqTarget(motor0, iq);
    } else {
      control0.iqCommand = 0.0f;
      setIqTarget(motor0, 0.0f);
    }
  }

  if (motor1Ready) {
    motor1.loopFOC();
    if (torqueAllowed && commandMotor1Enabled) {
      const float iq = runPositionHoldPd(motor1, control1, runtimeConfig1);
      setIqTarget(motor1, iq);
    } else {
      control1.iqCommand = 0.0f;
      setIqTarget(motor1, 0.0f);
    }
  }

  controlLoopCounter++;
  if ((nowUs - lastRuntimePublishUs) >= RUNTIME_STATE_PUBLISH_INTERVAL_US) {
    retrySkippedMotorsIfDue();

    const uint32_t loopDelta = controlLoopCounter - lastRuntimePublishLoopCounter;
    const uint32_t elapsedUs = lastRuntimePublishUs == 0 ? 0 : nowUs - lastRuntimePublishUs;
    if (loopDelta > 0 && elapsedUs > 0) {
      lastControlLoopUs = (float)elapsedUs / (float)loopDelta;
    }
    lastRuntimePublishLoopCounter = controlLoopCounter;
    lastRuntimePublishUs = nowUs;
    publishRuntimeState();
  }
}

void setup() {
  Serial.begin(115200);
  const uint32_t serialStartMs = millis();
  while (!Serial && (millis() - serialStartMs) < SERIAL_STARTUP_WAIT_MS) {}

  deselectSpiSlaves();

  sharedMemoryBarrier();
  interfaceCoreReady = true;
}

void loop() {
  readUsbCommandPackets();

  RuntimeControlState runtimeState;
  const bool hasRuntimeState = readLatestRuntimeControlState(runtimeState);

  if (hasRuntimeState) {
    writeUsbStatePacketIfDue(runtimeState);
  }

  if (INTERFACE_IDLE_US > 0) {
    delayMicroseconds(INTERFACE_IDLE_US);
  }
}

void setup1() {
  while (!interfaceCoreReady) {
    delayMicroseconds(10);
  }
  controlSetup();
}

void loop1() {
  while (true) {
    controlStep();
  }
}
