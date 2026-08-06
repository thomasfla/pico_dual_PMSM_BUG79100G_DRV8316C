#pragma once

#include <Arduino.h>
#include <SimpleFOC.h>

#define GPIO_M0_PWM_A 0
#define GPIO_M0_PWM_B 1
#define GPIO_M0_PWM_C 2
#define GPIO_M1_PWM_A 3
#define GPIO_M1_PWM_B 4
#define GPIO_M1_PWM_C 5

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
#define GPIO_ADC_SYNC_PWM 27

#ifndef CURRENT_CONTROL_BANDWIDTH_HZ
#define CURRENT_CONTROL_BANDWIDTH_HZ 200.0f
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

#ifndef M0_SENSOR_OFFSET
#define M0_SENSOR_OFFSET 0.0f
#endif

#ifndef M1_SENSOR_OFFSET
#define M1_SENSOR_OFFSET 0.0f
#endif

#ifndef REQUIRE_ENCODER_STARTUP_HEALTH
#define REQUIRE_ENCODER_STARTUP_HEALTH 0
#endif

struct PositionHoldConfig {
  float iqLimit;
  float kp;
  float kd;
  int8_t sensorDirectionSign;
  float zeroElectricAngle;
  float sensorOffset;
};

static constexpr int GM3506_POLE_PAIRS = 11;
static constexpr float GM3506_PHASE_RESISTANCE_OHM = 5.50f / 2.0f;
static constexpr float GM3506_PHASE_INDUCTANCE_H = 0.00108f;
static constexpr float GM3506_PEAK_CURRENT_A = 3.0f;

static constexpr float SUPPLY_VOLTAGE_FALLBACK = 10.0f;
static constexpr float CURRENT_FOC_VOLTAGE_LIMIT = 10.0f;
static constexpr float POSITION_SENSOR_ALIGN_VOLTAGE = 1.0f;
static constexpr float DRIVER_VOLTAGE_LIMIT = 10.0f;
static constexpr float DRIVER_VOLTAGE_LIMIT_BUS_FRACTION = 0.95f;
static constexpr long PWM_FREQUENCY = 20000;

static constexpr float M0_POSITION_IQ_LIMIT_A = GM3506_PEAK_CURRENT_A;
static constexpr float M1_POSITION_IQ_LIMIT_A = GM3506_PEAK_CURRENT_A;
static constexpr float M0_POSITION_PD_KP_A_PER_RAD = 0.65f;
static constexpr float M0_POSITION_PD_KD_A_PER_RAD_PER_S = 0.0f;
static constexpr float M1_POSITION_PD_KP_A_PER_RAD = 0.65f;
static constexpr float M1_POSITION_PD_KD_A_PER_RAD_PER_S = 0.0f;
static constexpr float POSITION_PD_KP_MIN_A_PER_RAD = 0.0f;
static constexpr float POSITION_PD_KP_MAX_A_PER_RAD = 100.0f;
static constexpr float POSITION_PD_KD_MIN_A_PER_RAD_PER_S = 0.0f;
static constexpr float POSITION_PD_KD_MAX_A_PER_RAD_PER_S = 10.0f;
static constexpr float POSITION_TARGET_VELOCITY_LIMIT_RAD_S = 200.0f;
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

static constexpr PositionHoldConfig MOTOR0_CONFIG = {
  M0_POSITION_IQ_LIMIT_A,
  M0_POSITION_PD_KP_A_PER_RAD,
  M0_POSITION_PD_KD_A_PER_RAD_PER_S,
  M0_SENSOR_DIRECTION_SIGN,
  M0_ZERO_ELECTRIC_ANGLE,
  M0_SENSOR_OFFSET,
};

static constexpr PositionHoldConfig MOTOR1_CONFIG = {
  M1_POSITION_IQ_LIMIT_A,
  M1_POSITION_PD_KP_A_PER_RAD,
  M1_POSITION_PD_KD_A_PER_RAD_PER_S,
  M1_SENSOR_DIRECTION_SIGN,
  M1_ZERO_ELECTRIC_ANGLE,
  M1_SENSOR_OFFSET,
};

static constexpr uint32_t ENCODER_SPI_HZ = 10000000;
static constexpr uint16_t ENCODER_POWERUP_DELAY_MS = 250;
static constexpr uint8_t ENCODER_STARTUP_READ_ATTEMPTS = 4;
static constexpr uint8_t ENCODER_HEALTH_READ_ATTEMPTS = 8;
static constexpr uint16_t ENCODER_HEALTH_RETRY_US = 200;
static constexpr uint16_t ENCODER_CPR = 16384;
static constexpr uint16_t ENCODER_MAG_MIN = 1000;
static constexpr uint16_t ENCODER_MAG_MAX = 14000;

static constexpr float ADC_SCK_HZ = 20000000.0f;
static constexpr float ADC_TRIGGER_DUTY = 1.0f - 0.045f;
static constexpr float CURRENT_SENSE_VREF = 3.3f;
static constexpr float ADC_FULL_SCALE_COUNTS = 4096.0f;
static constexpr float ADC_ZERO_CURRENT_COUNTS = ADC_FULL_SCALE_COUNTS * 0.5f;
static constexpr float DRV_CSA_GAIN_V_PER_A = 0.375f;
static constexpr float ADC_COUNT_TO_PHASE_CURRENT_A =
  CURRENT_SENSE_VREF / ADC_FULL_SCALE_COUNTS / DRV_CSA_GAIN_V_PER_A;
static constexpr uint16_t CURRENT_SENSE_CALIBRATION_SAMPLES = 128;
static constexpr uint16_t CURRENT_SENSE_CALIBRATION_SAMPLE_US = 50;

static constexpr uint8_t VBUS_ADC_BITS = 12;
static constexpr float VBUS_ADC_MAX_COUNTS = (1u << VBUS_ADC_BITS) - 1u;
static constexpr float VBUS_DIVIDER_HIGH_OHM = 100000.0f;
static constexpr float VBUS_DIVIDER_LOW_OHM = 10000.0f;
static constexpr float VBUS_DIVIDER_RATIO =
  (VBUS_DIVIDER_HIGH_OHM + VBUS_DIVIDER_LOW_OHM) / VBUS_DIVIDER_LOW_OHM;
static constexpr uint16_t VBUS_STARTUP_SAMPLES = 16;

static constexpr uint32_t SERIAL_STARTUP_WAIT_MS = 250;
static constexpr uint32_t CALIBRATION_ENTRY_WAIT_MS = 5000;
static constexpr size_t CALIBRATION_EEPROM_BYTES = 256;
static constexpr uint32_t STARTUP_TARGET_SETTLE_MS = 50;
static constexpr uint32_t RUNTIME_STATE_PUBLISH_INTERVAL_US = 1000;
static constexpr uint32_t USB_STATE_FRAME_INTERVAL_US = 1000;
static constexpr uint32_t MOTOR_START_RETRY_INTERVAL_MS = 500;
static constexpr uint32_t INTERFACE_IDLE_US = 100;

static constexpr uint8_t USB_PACKET_MAGIC0 = 0xA5;
static constexpr uint8_t USB_PACKET_MAGIC1 = 0x5A;
static constexpr uint8_t USB_PACKET_VERSION = 1;
static constexpr uint8_t USB_PACKET_TYPE_COMMAND = 0x43;  // 'C'
static constexpr uint8_t USB_PACKET_TYPE_STATE = 0x53;    // 'S'
static constexpr uint8_t USB_STATE_FLAG_M0_READY = 1u << 0;
static constexpr uint8_t USB_STATE_FLAG_M1_READY = 1u << 1;
