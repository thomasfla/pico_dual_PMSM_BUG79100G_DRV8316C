// Dual DRV8316C + GM3506 current-FOC position hold.
// SimpleFOC handles FOC commutation; this file supplies a small PD loop that
// commands Iq in closed-loop current control.

#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include <stdint.h>
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/spi.h"
#include "BU79100QuadReader.h"
#include "drivers/drv8316/drv8316.h"

#define GPIO_M0_PWM_A 0
#define GPIO_M0_PWM_B 1
#define GPIO_M0_PWM_C 2

#define GPIO_M1_PWM_A 3
#define GPIO_M1_PWM_B 4
#define GPIO_M1_PWM_C 5

// BU79100G ADC interface. GPIO6/7 drive both motor ADC banks through series resistors.
#define GPIO_ADC_SCK 6
#define GPIO_ADC_CSB 7

#define GPIO_M0_ADC_DATA_A 8
#define GPIO_M0_ADC_DATA_C 9
#define GPIO_M1_ADC_DATA_A 10
#define GPIO_M1_ADC_DATA_C 11

#define GPIO_M0_ENC_CS 12
#define GPIO_M1_ENC_CS 13

#define GPIO_M0_DRV_CS 14
#define GPIO_M1_DRV_CS 15

#define GPIO_SPI0_MISO 16
#define GPIO_DRV_Mx_nFAULT 17
#define GPIO_SPI0_CLK 18
#define GPIO_SPI0_MOSI 19

#define GPIO_VBUS_SENSE 26

// Pin used later by the PIO current-sense state machine trigger.
#define GPIO_ADC_SYNC_PWM 27

#ifndef CURRENT_CONTROL_BANDWIDTH_HZ
#define CURRENT_CONTROL_BANDWIDTH_HZ 200.0f
#endif

#ifndef TIMING_PROFILE_ENABLED
#define TIMING_PROFILE_ENABLED 1
#endif

#ifndef BINARY_TELEMETRY_ENABLED
#define BINARY_TELEMETRY_ENABLED 1
#endif

#ifndef M0_SENSOR_DIRECTION_SIGN
#define M0_SENSOR_DIRECTION_SIGN 0
#endif

#ifndef M1_SENSOR_DIRECTION_SIGN
#define M1_SENSOR_DIRECTION_SIGN 0
#endif

#ifndef M0_ZERO_ELECTRIC_ANGLE
#define M0_ZERO_ELECTRIC_ANGLE NOT_SET
#endif

#ifndef M1_ZERO_ELECTRIC_ANGLE
#define M1_ZERO_ELECTRIC_ANGLE NOT_SET
#endif

static constexpr int GM3506_POLE_PAIRS = 11;
static constexpr float GM3506_PHASE_RESISTANCE_OHM = 5.50f / 2.0f;  // Wye motor: phase-to-phase / 2.
// Starting value for 3506-class gimbal motors; measure your exact winding if you need precise bandwidth.
static constexpr float GM3506_PHASE_INDUCTANCE_H = 0.00108f;
static constexpr float SUPPLY_VOLTAGE_FALLBACK = 10.0f;
static constexpr float CURRENT_FOC_VOLTAGE_LIMIT = 10.0f;
static constexpr float POSITION_SENSOR_ALIGN_VOLTAGE = 1.0f;
static constexpr float DRIVER_VOLTAGE_LIMIT = 10.0f;
static constexpr float DRIVER_VOLTAGE_LIMIT_BUS_FRACTION = 0.95f;
static constexpr float GM3506_CONTINUOUS_CURRENT_A = 1.0f;
static constexpr float GM3506_PEAK_CURRENT_A = 3.0f;
static constexpr float M0_POSITION_IQ_LIMIT_A = GM3506_PEAK_CURRENT_A;
static constexpr float M1_POSITION_IQ_LIMIT_A = GM3506_PEAK_CURRENT_A;
static constexpr float M0_POSITION_PD_KP_A_PER_RAD = 0.65f;
static constexpr float M0_POSITION_PD_KD_A_PER_RAD_PER_S = 0.0f;
static constexpr float M1_POSITION_PD_KP_A_PER_RAD = 0.65f;
static constexpr float M1_POSITION_PD_KD_A_PER_RAD_PER_S = 0.0f;
static constexpr float POSITION_VELOCITY_FILTER_TF = 0.04f;
static constexpr float CURRENT_CONTROL_BANDWIDTH_RAD_PER_S =
  6.28318530718f * CURRENT_CONTROL_BANDWIDTH_HZ;
static constexpr float CURRENT_CONTROL_P =
  GM3506_PHASE_INDUCTANCE_H * CURRENT_CONTROL_BANDWIDTH_RAD_PER_S;
static constexpr float CURRENT_CONTROL_I =
  GM3506_PHASE_RESISTANCE_OHM * CURRENT_CONTROL_BANDWIDTH_RAD_PER_S;
static constexpr float CURRENT_CONTROL_D = 0.0f;
static constexpr float CURRENT_CONTROL_RAMP = 0.0f;
static constexpr float CURRENT_CONTROL_FILTER_TF = 0.0002f;
static constexpr uint32_t ENCODER_SPI_HZ = 10000000;
static constexpr uint8_t ENCODER_STARTUP_READ_ATTEMPTS = 4;
static constexpr uint16_t ENCODER_CPR = 16384;
static constexpr uint16_t ENCODER_MAG_MIN = 1000;
static constexpr uint16_t ENCODER_MAG_MAX = 14000;
static constexpr long PWM_FREQUENCY = 20000;
static constexpr float ADC_SCK_HZ = 20000000.0f;
static constexpr float ADC_TRIGGER_DUTY = 1.0f - 0.045f;
static constexpr float CURRENT_SENSE_VREF = 3.3f;  // Pico 3V3 feeds DRV VREF/ILIM and the BU79100 ADCs.
static constexpr float ADC_FULL_SCALE_COUNTS = 4096.0f;
static constexpr float ADC_ZERO_CURRENT_COUNTS = ADC_FULL_SCALE_COUNTS * 0.5f;
static constexpr float DRV_CSA_GAIN_V_PER_A = 0.375f;  // DRV8316_CSAGain::Gain_0V375.
static constexpr float ADC_COUNT_TO_PHASE_CURRENT_A =
  CURRENT_SENSE_VREF / ADC_FULL_SCALE_COUNTS / DRV_CSA_GAIN_V_PER_A;
static constexpr uint8_t VBUS_ADC_BITS = 12;
static constexpr float VBUS_ADC_MAX_COUNTS = (1u << VBUS_ADC_BITS) - 1u;
static constexpr float VBUS_DIVIDER_HIGH_OHM = 100000.0f;
static constexpr float VBUS_DIVIDER_LOW_OHM = 10000.0f;
static constexpr float VBUS_DIVIDER_RATIO =
  (VBUS_DIVIDER_HIGH_OHM + VBUS_DIVIDER_LOW_OHM) / VBUS_DIVIDER_LOW_OHM;
static constexpr uint16_t VBUS_STARTUP_SAMPLES = 16;
static constexpr uint32_t SERIAL_STARTUP_WAIT_MS = 250;
static constexpr uint32_t STARTUP_TARGET_SETTLE_MS = 50;
static constexpr uint16_t CURRENT_SENSE_CALIBRATION_SAMPLES = 128;
static constexpr uint16_t CURRENT_SENSE_CALIBRATION_SAMPLE_US = 50;
static constexpr uint32_t TELEMETRY_FRAME_INTERVAL_US = 200;
static constexpr uint32_t TIMING_PROFILE_PRINT_INTERVAL_MS = 1000;
static constexpr uint8_t TELEMETRY_MAGIC0 = 0xA5;
static constexpr uint8_t TELEMETRY_MAGIC1 = 0x5A;
static constexpr uint8_t TELEMETRY_VERSION = 1;
static constexpr float TELEMETRY_CURRENT_SCALE_MA = 1000.0f;
static constexpr float TELEMETRY_POSITION_SCALE_URAD = 1000000.0f;
static constexpr float TELEMETRY_VELOCITY_SCALE_MRAD_S = 1000.0f;
static constexpr uint8_t TELEMETRY_FLAG_M0_READY = 1u << 0;
static constexpr uint8_t TELEMETRY_FLAG_M1_READY = 1u << 1;

static PhaseCurrent_s readMotor0Currents();
static PhaseCurrent_s readMotor1Currents();

#if TIMING_PROFILE_ENABLED
struct TimingStats {
  uint32_t count = 0;
  uint64_t totalUs = 0;
  uint32_t minUs = UINT32_MAX;
  uint32_t maxUs = 0;
};

struct TimingProfile {
  TimingStats adcRead;
  TimingStats encoderRead;
  TimingStats foc;
  TimingStats positionControl;
  TimingStats twoMotorControl;
};

static TimingProfile timingProfile;
static uint32_t lastTimingProfilePrintMs = 0;

static void addTimingSample(TimingStats &stats, uint32_t elapsedUs) {
  stats.count++;
  stats.totalUs += elapsedUs;
  if (elapsedUs < stats.minUs) {
    stats.minUs = elapsedUs;
  }
  if (elapsedUs > stats.maxUs) {
    stats.maxUs = elapsedUs;
  }
}

static void resetTimingStats(TimingStats &stats) {
  stats.count = 0;
  stats.totalUs = 0;
  stats.minUs = UINT32_MAX;
  stats.maxUs = 0;
}

static void resetTimingProfile() {
  resetTimingStats(timingProfile.adcRead);
  resetTimingStats(timingProfile.encoderRead);
  resetTimingStats(timingProfile.foc);
  resetTimingStats(timingProfile.positionControl);
  resetTimingStats(timingProfile.twoMotorControl);
}

static void printTimingStats(const char *label, const TimingStats &stats) {
  Serial.print(' ');
  Serial.print(label);
  Serial.print("_n=");
  Serial.print(stats.count);
  Serial.print(" avg=");
  if (stats.count > 0) {
    Serial.print((float)stats.totalUs / (float)stats.count, 2);
    Serial.print(" min=");
    Serial.print(stats.minUs);
    Serial.print(" max=");
    Serial.print(stats.maxUs);
  } else {
    Serial.print("nan min=0 max=0");
  }
}

static void printTimingProfile() {
  Serial.print("TIMING us");
  printTimingStats("adc", timingProfile.adcRead);
  printTimingStats("enc", timingProfile.encoderRead);
  printTimingStats("foc", timingProfile.foc);
  printTimingStats("posvel", timingProfile.positionControl);
  printTimingStats("ctrl2", timingProfile.twoMotorControl);
  Serial.println();
  resetTimingProfile();
}
#endif

struct PositionHoldState {
  float holdAngle = 0.0f;
  float iqCommand = 0.0f;
};

struct PositionHoldConfig {
  float iqLimit;
  float kp;
  float kd;
  int8_t sensorDirectionSign;
  float zeroElectricAngle;
};

struct BusVoltageReading {
  float voltage = 0.0f;
  float rawCounts = 0.0f;
};

struct EncoderRegisterDiagnostics {
  uint16_t magnitude = 0;
  uint16_t diagnostics = 0;
  bool magnitudeOk = false;
  bool diagnosticsOk = false;
};

struct __attribute__((packed)) TelemetryFrame {
  uint8_t magic0;
  uint8_t magic1;
  uint8_t version;
  uint8_t length;
  uint16_t sequence;
  uint32_t t_us;
  int16_t m0_phase_a_mA;
  int16_t m0_phase_b_mA;
  int16_t m0_phase_c_mA;
  int16_t m0_id_mA;
  int16_t m0_iq_mA;
  int16_t m0_iq_target_mA;
  int16_t m1_phase_a_mA;
  int16_t m1_phase_b_mA;
  int16_t m1_phase_c_mA;
  int16_t m1_id_mA;
  int16_t m1_iq_mA;
  int16_t m1_iq_target_mA;
  int32_t m0_position_urad;
  int32_t m0_velocity_mrad_s;
  int32_t m1_position_urad;
  int32_t m1_velocity_mrad_s;
  uint8_t flags;
  uint8_t checksum;
};

static_assert(sizeof(TelemetryFrame) == 52, "Unexpected telemetry frame size");

class CheckedAS5048ASensor : public Sensor {
public:
  explicit CheckedAS5048ASensor(uint8_t csPin)
    : csPin_(csPin), settings_(ENCODER_SPI_HZ, MSBFIRST, SPI_MODE1) {}

  void init(SPIClass *spi = &SPI) {
    spi_ = spi;
    pinMode(csPin_, OUTPUT);
    digitalWrite(csPin_, HIGH);
    spi_->begin();

    transfer16(makeReadCommand(AS5048A_ANGLE_REG));
    delayMicroseconds(5);

    uint16_t raw = 0;
    for (uint8_t i = 0; i < ENCODER_STARTUP_READ_ATTEMPTS; i++) {
      if (readRawChecked(raw)) {
        acceptRaw(raw);
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

    spi_->beginTransaction(settings_);
    spi_->endTransaction();
    spiHw_ = spiHw;
    spi_set_baudrate(spiHw_, ENCODER_SPI_HZ);
    spi_set_format(spiHw_, 16, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
    fastRuntimeSpi_ = true;
  }

  float getSensorAngle() override {
    uint16_t raw = 0;
#if TIMING_PROFILE_ENABLED
    const uint32_t startUs = micros();
#endif
    if (!readRawChecked(raw)) {
#if TIMING_PROFILE_ENABLED
      addTimingSample(timingProfile.encoderRead, micros() - startUs);
#endif
      return -1.0f;
    }
#if TIMING_PROFILE_ENABLED
    addTimingSample(timingProfile.encoderRead, micros() - startUs);
#endif

    acceptRaw(raw);
    return rawToAngle(raw);
  }

  uint16_t raw() const {
    return lastRaw_;
  }

  EncoderRegisterDiagnostics readRegisterDiagnostics() {
    EncoderRegisterDiagnostics result;
    result.magnitudeOk = readRegisterData(AS5048A_MAGNITUDE_REG, result.magnitude);
    result.diagnosticsOk = readRegisterData(AS5048A_DIAGNOSTICS_REG, result.diagnostics);
    return result;
  }

private:
  static constexpr uint16_t AS5048A_ANGLE_REG = 0x3FFF;
  static constexpr uint16_t AS5048A_MAGNITUDE_REG = 0x3FFE;
  static constexpr uint16_t AS5048A_DIAGNOSTICS_REG = 0x3FFD;
  static constexpr uint16_t AS5048A_RESULT_MASK = 0x3FFF;
  static constexpr uint16_t AS5048A_READ_BIT = 0x4000;
  static constexpr uint16_t AS5048A_PARITY_BIT = 0x8000;
  static constexpr uint16_t AS5048A_ERROR_FLAG = 0x4000;

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
      return false;
    }

    const uint16_t candidate = frame & AS5048A_RESULT_MASK;
    raw = candidate;
    return true;
  }

  void acceptRaw(uint16_t raw) {
    lastRaw_ = raw;
  }

  uint8_t csPin_;
  SPIClass *spi_ = &SPI;
  spi_inst_t *spiHw_ = nullptr;
  SPISettings settings_;
  uint16_t lastRaw_ = 0;
  bool fastRuntimeSpi_ = false;
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
#if TIMING_PROFILE_ENABLED
    const uint32_t startUs = micros();
#endif
    PhaseCurrent_s current = readCallback_();
#if TIMING_PROFILE_ENABLED
    addTimingSample(timingProfile.adcRead, micros() - startUs);
#endif
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

static constexpr PositionHoldConfig motor0Config = {
  M0_POSITION_IQ_LIMIT_A,
  M0_POSITION_PD_KP_A_PER_RAD,
  M0_POSITION_PD_KD_A_PER_RAD_PER_S,
  M0_SENSOR_DIRECTION_SIGN,
  M0_ZERO_ELECTRIC_ANGLE,
};

static constexpr PositionHoldConfig motor1Config = {
  M1_POSITION_IQ_LIMIT_A,
  M1_POSITION_PD_KP_A_PER_RAD,
  M1_POSITION_PD_KD_A_PER_RAD_PER_S,
  M1_SENSOR_DIRECTION_SIGN,
  M1_ZERO_ELECTRIC_ANGLE,
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

static uint32_t lastTelemetryFrameUs = 0;
static uint16_t telemetrySequence = 0;
static PositionHoldState control0;
static PositionHoldState control1;
static bool motor0Ready = false;
static bool motor1Ready = false;

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

static BusVoltageReading readBusVoltage() {
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
  return {
    adcVoltage * VBUS_DIVIDER_RATIO,
    rawCounts,
  };
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

static const char *directionName(Direction direction) {
  switch (direction) {
    case Direction::CW:
      return "CW";
    case Direction::CCW:
      return "CCW";
    default:
      return "UNKNOWN";
  }
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

static uint8_t encoderAgc(uint16_t diagnostics) {
  return diagnostics & 0x00FF;
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

static bool startClosedLoopMotor(BLDCMotor &motor, PositionHoldState &control, const char *name) {
  const int focOk = motor.initFOC();
  if (focOk) {
    control.holdAngle = motor.shaft_angle;
    control.iqCommand = 0.0f;
    motor.target = 0.0f;
    motor.current_sp = 0.0f;
    Serial.print(name);
    Serial.print(" current-FOC hold ready, angle=");
    Serial.println(control.holdAngle, 5);
    Serial.print(name);
    Serial.print(" alignment dir=");
    Serial.print(directionName(motor.sensor_direction));
    Serial.print(" zero_electric_angle=");
    Serial.println(motor.zero_electric_angle, 6);
    return true;
  } else {
    Serial.print(name);
    Serial.println(" initFOC failed");
    motor.disable();
    return false;
  }
}

static void setIqTarget(BLDCMotor &motor, float iq) {
  motor.target = iq;
  motor.current_sp = iq;
}

static float runPositionHoldPd(
  BLDCMotor &motor,
  PositionHoldState &control,
  const PositionHoldConfig &config
) {
  const float angle = motor.shaftAngle();
  const float velocity = motor.shaftVelocity();
  const float angleError = control.holdAngle - angle;
  const float iq = config.kp * angleError - config.kd * velocity;

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
    control0.holdAngle = motor0.shaftAngle();
    control0.iqCommand = 0.0f;
  }
  if (motor1Ready) {
    control1.holdAngle = motor1.shaftAngle();
    control1.iqCommand = 0.0f;
  }
}

static int32_t scaleToI32(float value, float scale) {
  const float scaled = value * scale;
  if (scaled > (float)INT32_MAX) {
    return INT32_MAX;
  }
  if (scaled < (float)INT32_MIN) {
    return INT32_MIN;
  }
  return (int32_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

static int16_t scaleToI16(float value, float scale) {
  const int32_t scaled = scaleToI32(value, scale);
  if (scaled > INT16_MAX) {
    return INT16_MAX;
  }
  if (scaled < INT16_MIN) {
    return INT16_MIN;
  }
  return (int16_t)scaled;
}

static uint8_t telemetryChecksum(const TelemetryFrame &frame) {
  const uint8_t *bytes = (const uint8_t *)&frame;
  uint8_t checksum = 0;
  for (size_t i = 0; i < (sizeof(TelemetryFrame) - 1); i++) {
    checksum ^= bytes[i];
  }
  return checksum;
}

static void writeTelemetryFrame() {
  if (!Serial || Serial.availableForWrite() < (int)sizeof(TelemetryFrame)) {
    return;
  }

  const PhaseCurrent_s m0 = currentSense0.lastPhaseCurrents();
  const PhaseCurrent_s m1 = currentSense1.lastPhaseCurrents();

  TelemetryFrame frame = {};
  frame.magic0 = TELEMETRY_MAGIC0;
  frame.magic1 = TELEMETRY_MAGIC1;
  frame.version = TELEMETRY_VERSION;
  frame.length = sizeof(TelemetryFrame);
  frame.sequence = telemetrySequence++;
  frame.t_us = micros();

  frame.m0_phase_a_mA = scaleToI16(m0.a, TELEMETRY_CURRENT_SCALE_MA);
  frame.m0_phase_b_mA = scaleToI16(m0.b, TELEMETRY_CURRENT_SCALE_MA);
  frame.m0_phase_c_mA = scaleToI16(m0.c, TELEMETRY_CURRENT_SCALE_MA);
  frame.m0_id_mA = scaleToI16(motor0.current.d, TELEMETRY_CURRENT_SCALE_MA);
  frame.m0_iq_mA = scaleToI16(motor0.current.q, TELEMETRY_CURRENT_SCALE_MA);
  frame.m0_iq_target_mA = scaleToI16(control0.iqCommand, TELEMETRY_CURRENT_SCALE_MA);

  frame.m1_phase_a_mA = scaleToI16(m1.a, TELEMETRY_CURRENT_SCALE_MA);
  frame.m1_phase_b_mA = scaleToI16(m1.b, TELEMETRY_CURRENT_SCALE_MA);
  frame.m1_phase_c_mA = scaleToI16(m1.c, TELEMETRY_CURRENT_SCALE_MA);
  frame.m1_id_mA = scaleToI16(motor1.current.d, TELEMETRY_CURRENT_SCALE_MA);
  frame.m1_iq_mA = scaleToI16(motor1.current.q, TELEMETRY_CURRENT_SCALE_MA);
  frame.m1_iq_target_mA = scaleToI16(control1.iqCommand, TELEMETRY_CURRENT_SCALE_MA);

  frame.m0_position_urad = scaleToI32(motor0Ready ? motor0.shaft_angle : 0.0f, TELEMETRY_POSITION_SCALE_URAD);
  frame.m0_velocity_mrad_s = scaleToI32(motor0Ready ? motor0.shaft_velocity : 0.0f, TELEMETRY_VELOCITY_SCALE_MRAD_S);
  frame.m1_position_urad = scaleToI32(motor1Ready ? motor1.shaft_angle : 0.0f, TELEMETRY_POSITION_SCALE_URAD);
  frame.m1_velocity_mrad_s = scaleToI32(motor1Ready ? motor1.shaft_velocity : 0.0f, TELEMETRY_VELOCITY_SCALE_MRAD_S);
  frame.flags = (motor0Ready ? TELEMETRY_FLAG_M0_READY : 0) |
    (motor1Ready ? TELEMETRY_FLAG_M1_READY : 0);
  frame.checksum = telemetryChecksum(frame);

  Serial.write((const uint8_t *)&frame, sizeof(frame));
}

static bool printStartupEncoderHealth(const char *name, CheckedAS5048ASensor &encoder) {
  const EncoderRegisterDiagnostics regs = encoder.readRegisterDiagnostics();
  const bool healthy = encoderDiagnosticsHealthy(regs);

  Serial.print(name);
  Serial.print(" encoder health=");
  Serial.print(healthy ? "ok" : "fail");
  Serial.print(" raw=");
  Serial.print(encoder.raw());
  Serial.print(" mag=");
  if (regs.magnitudeOk) {
    Serial.print(regs.magnitude);
  } else {
    Serial.print("bad");
  }
  Serial.print(" diag=");
  if (regs.diagnosticsOk) {
    Serial.print("0x");
    Serial.print(regs.diagnostics, HEX);
    Serial.print(" agc=");
    Serial.print(encoderAgc(regs.diagnostics));
    Serial.print(" ocf=");
    Serial.print(encoderOcf(regs.diagnostics));
    Serial.print(" cof=");
    Serial.print(encoderCof(regs.diagnostics));
    Serial.print(" low=");
    Serial.print(encoderCompLow(regs.diagnostics));
    Serial.print(" high=");
    Serial.print(encoderCompHigh(regs.diagnostics));
  } else {
    Serial.print("bad");
  }
  Serial.println();
  return healthy;
}

void setup() {
  const uint32_t setupStartUs = micros();
  // USB CDC ignores this as a transport limit; the host still expects a line coding value.
  Serial.begin(115200);
  const uint32_t serialStartMs = millis();
  while (!Serial && (millis() - serialStartMs) < SERIAL_STARTUP_WAIT_MS) {}

  deselectSpiSlaves();
  pinMode(GPIO_DRV_Mx_nFAULT, INPUT_PULLUP);
  configureSpiPins();

  const BusVoltageReading busVoltage = readBusVoltage();
  Serial.print("VBUS=");
  Serial.print(busVoltage.voltage, 2);
  Serial.print(" V raw=");
  Serial.println(busVoltage.rawCounts, 1);

  encoder0.init(&SPI);
  Serial.println("M0 AS5048A encoder started");
  encoder1.init(&SPI);
  Serial.println("M1 AS5048A encoder started");
  Serial.print("AS5048A SPI hz=");
  Serial.println(ENCODER_SPI_HZ);
  const bool encoder0Healthy = printStartupEncoderHealth("M0", encoder0);
  const bool encoder1Healthy = printStartupEncoderHealth("M1", encoder1);

  configureDriver(driver0, busVoltage.voltage);
  configureDriver(driver1, busVoltage.voltage);
  encoder0.enableFastRuntimeSpi(spi0);
  encoder1.enableFastRuntimeSpi(spi0);

  configureAdcTriggerPwm();
  syncAllPwmSlices();
  const bool currentAdcOk = currentAdc.init(ADC_SCK_HZ);
  Serial.println(currentAdcOk ? "BU79100 PIO ADC started" : "BU79100 PIO ADC init failed");

  currentSense0.linkDriver(&driver0);
  currentSense1.linkDriver(&driver1);
  currentSense0.skip_align = true;
  currentSense1.skip_align = true;
  const bool currentSense0Ok = currentSense0.init();
  const bool currentSense1Ok = currentSense1.init();
  const bool currentFeedback0Ok = currentAdcOk && currentSense0Ok;
  const bool currentFeedback1Ok = currentAdcOk && currentSense1Ok;
  Serial.print("M0 current sense=");
  Serial.println(currentFeedback0Ok ? "ok" : "fail");
  Serial.print("M1 current sense=");
  Serial.println(currentFeedback1Ok ? "ok" : "fail");
  Serial.print("Current loop target=");
  Serial.print(CURRENT_CONTROL_BANDWIDTH_HZ, 1);
  Serial.print(" Hz P=");
  Serial.print(CURRENT_CONTROL_P, 3);
  Serial.print(" I=");
  Serial.print(CURRENT_CONTROL_I, 1);
  Serial.print(" LPF_Tf_us=");
  Serial.print(CURRENT_CONTROL_FILTER_TF * 1000000.0f, 0);
  Serial.print(" ramp=");
  Serial.println(CURRENT_CONTROL_RAMP, 1);
  Serial.print("Torque limits: continuous_current=");
  Serial.print(GM3506_CONTINUOUS_CURRENT_A, 2);
  Serial.print(" A peak_current=");
  Serial.print(GM3506_PEAK_CURRENT_A, 2);
  Serial.print(" A current_foc_v_limit=");
  Serial.print(CURRENT_FOC_VOLTAGE_LIMIT, 2);
  Serial.print(" V effective_vq_limit=");
  Serial.print(driver0.voltage_limit, 2);
  Serial.print(" V supply=");
  Serial.print(driver0.voltage_power_supply, 2);
  Serial.println(" V");

  Serial.println("Torque mode=foc_current");

  if (encoder0Healthy && currentFeedback0Ok) {
    if (configureMotor(motor0, driver0, currentSense0, encoder0, motor0Config)) {
      motor0Ready = startClosedLoopMotor(motor0, control0, "M0");
    } else {
      driver0.disable();
      Serial.println("M0 motor skipped: motor init failed");
    }
  } else {
    driver0.disable();
    Serial.println(encoder0Healthy ?
      "M0 motor skipped: current feedback unavailable" :
      "M0 motor skipped: encoder unhealthy");
  }

  if (encoder1Healthy && currentFeedback1Ok) {
    if (configureMotor(motor1, driver1, currentSense1, encoder1, motor1Config)) {
      motor1Ready = startClosedLoopMotor(motor1, control1, "M1");
    } else {
      driver1.disable();
      Serial.println("M1 motor skipped: motor init failed");
    }
  } else {
    driver1.disable();
    Serial.println(encoder1Healthy ?
      "M1 motor skipped: current feedback unavailable" :
      "M1 motor skipped: encoder unhealthy");
  }
  settleStartupTargets(STARTUP_TARGET_SETTLE_MS);

  Serial.println("Current FOC position hold started");
  Serial.print("Binary telemetry=");
  Serial.print(BINARY_TELEMETRY_ENABLED ? "enabled" : "disabled");
  Serial.print(" v");
  Serial.print(TELEMETRY_VERSION);
  Serial.print(" frame_size=");
  Serial.print(sizeof(TelemetryFrame));
  Serial.print(" interval_us=");
  Serial.println(TELEMETRY_FRAME_INTERVAL_US);
  Serial.print("Setup elapsed_ms=");
  Serial.println((micros() - setupStartUs) * 0.001f, 1);
#if TIMING_PROFILE_ENABLED
  resetTimingProfile();
  lastTimingProfilePrintMs = millis();
#endif
}

void loop() {
#if TIMING_PROFILE_ENABLED
  const uint32_t controlStartUs = micros();
#endif

  if (motor0Ready) {
#if TIMING_PROFILE_ENABLED
    uint32_t startUs = micros();
#endif
    motor0.loopFOC();
#if TIMING_PROFILE_ENABLED
    addTimingSample(timingProfile.foc, micros() - startUs);
    startUs = micros();
#endif
    const float iq = runPositionHoldPd(motor0, control0, motor0Config);
    setIqTarget(motor0, iq);
#if TIMING_PROFILE_ENABLED
    addTimingSample(timingProfile.positionControl, micros() - startUs);
#endif
  }

  if (motor1Ready) {
#if TIMING_PROFILE_ENABLED
    uint32_t startUs = micros();
#endif
    motor1.loopFOC();
#if TIMING_PROFILE_ENABLED
    addTimingSample(timingProfile.foc, micros() - startUs);
    startUs = micros();
#endif
    const float iq = runPositionHoldPd(motor1, control1, motor1Config);
    setIqTarget(motor1, iq);
#if TIMING_PROFILE_ENABLED
    addTimingSample(timingProfile.positionControl, micros() - startUs);
#endif
  }

#if TIMING_PROFILE_ENABLED
  if (motor0Ready || motor1Ready) {
    addTimingSample(timingProfile.twoMotorControl, micros() - controlStartUs);
  }
#endif

#if BINARY_TELEMETRY_ENABLED
  const uint32_t nowUs = micros();
  if ((nowUs - lastTelemetryFrameUs) >= TELEMETRY_FRAME_INTERVAL_US) {
    lastTelemetryFrameUs = nowUs;
    writeTelemetryFrame();
  }
#endif

#if TIMING_PROFILE_ENABLED
  const uint32_t nowMs = millis();
  if ((nowMs - lastTimingProfilePrintMs) >= TIMING_PROFILE_PRINT_INTERVAL_MS) {
    lastTimingProfilePrintMs = nowMs;
    printTimingProfile();
  }
#endif
}
