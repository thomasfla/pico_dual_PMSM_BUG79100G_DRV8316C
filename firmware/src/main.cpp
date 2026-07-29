// Dual DRV8316C + GM3506 open-loop velocity test.
// Uses SimpleFOC 3PWM mode and starts both motors directly at boot.

#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "current_sense/GenericCurrentSense.h"
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

// Pin used later by the PIO current-sense state machine trigger.
#define GPIO_ADC_SYNC_PWM 27

static constexpr int GM3506_POLE_PAIRS = 11;
static constexpr float SUPPLY_VOLTAGE = 7.0f;
static constexpr float OPEN_LOOP_VOLTAGE_LIMIT = 1.0f;
static constexpr float DRIVER_VOLTAGE_LIMIT = 3.0f * OPEN_LOOP_VOLTAGE_LIMIT;
static constexpr float TARGET_VELOCITY = 0.05f;  // rad/s
static constexpr long PWM_FREQUENCY = 20000;
static constexpr float ADC_SCK_HZ = 20000000.0f;
static constexpr float ADC_TRIGGER_DUTY = 1.0f - 0.045f;
static constexpr float ADC_COUNTS_TO_CURRENT = 1.0f;  // Calibrated counts until A/count is measured.
static constexpr uint32_t CURRENT_PRINT_INTERVAL_MS = 1;

static PhaseCurrent_s readMotor0Currents();
static PhaseCurrent_s readMotor1Currents();

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
BU79100QuadReader currentAdc(pio0, GPIO_ADC_SCK, GPIO_ADC_CSB, GPIO_M0_ADC_DATA_A, GPIO_ADC_SYNC_PWM);
GenericCurrentSense currentSense0(readMotor0Currents);
GenericCurrentSense currentSense1(readMotor1Currents);

static uint32_t lastCurrentPrintMs = 0;

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

static void configureDriver(DRV8316Driver3PWM &driver) {
  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.voltage_limit = DRIVER_VOLTAGE_LIMIT;
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

  driver.setCurrentSenseGain(DRV8316_CSAGain::Gain_0V15);
  delayMicroseconds(5);

  driver.setDriverOffEnabled(false);
  delayMicroseconds(5);

  driver.clearFault();
}

static PhaseCurrent_s readMotor0Currents() {
  const BU79100QuadSample sample = currentAdc.read();
  return {
    (float)sample.raw[0] * ADC_COUNTS_TO_CURRENT,
    0.0f,
    (float)sample.raw[1] * ADC_COUNTS_TO_CURRENT,
  };
}

static PhaseCurrent_s readMotor1Currents() {
  const BU79100QuadSample sample = currentAdc.read();
  return {
    (float)sample.raw[2] * ADC_COUNTS_TO_CURRENT,
    0.0f,
    (float)sample.raw[3] * ADC_COUNTS_TO_CURRENT,
  };
}

static void configureMotor(BLDCMotor &motor, DRV8316Driver3PWM &driver, CurrentSense &currentSense) {
  motor.linkDriver(&driver);
  motor.linkCurrentSense(&currentSense);
  motor.voltage_limit = OPEN_LOOP_VOLTAGE_LIMIT;
  motor.velocity_limit = fabs(TARGET_VELOCITY);
  motor.controller = MotionControlType::velocity_openloop;
  motor.foc_modulation = FOCModulationType::SinePWM;
  motor.init();
}

static void printCurrents() {
  const BU79100QuadSample raw = currentAdc.read();
  const PhaseCurrent_s m0 = currentSense0.getPhaseCurrents();
  const PhaseCurrent_s m1 = currentSense1.getPhaseCurrents();

  Serial.print("raw=");
  Serial.print(raw.raw[0]);
  Serial.print(',');
  Serial.print(raw.raw[1]);
  Serial.print(',');
  Serial.print(raw.raw[2]);
  Serial.print(',');
  Serial.print(raw.raw[3]);

  Serial.print(" M0[A,Bcalc,C]=");
  Serial.print(m0.a, 1);
  Serial.print(',');
  Serial.print(-m0.a - m0.c, 1);
  Serial.print(',');
  Serial.print(m0.c, 1);

  Serial.print(" M1[A,Bcalc,C]=");
  Serial.print(m1.a, 1);
  Serial.print(',');
  Serial.print(-m1.a - m1.c, 1);
  Serial.print(',');
  Serial.println(m1.c, 1);
}

void setup() {
  Serial.begin(115200);

  deselectSpiSlaves();
  pinMode(GPIO_DRV_Mx_nFAULT, INPUT_PULLUP);
  configureSpiPins();

  configureDriver(driver0);
  configureDriver(driver1);

  configureAdcTriggerPwm();
  syncAllPwmSlices();
  const bool currentAdcOk = currentAdc.init(ADC_SCK_HZ);
  Serial.println(currentAdcOk ? "BU79100 PIO ADC started" : "BU79100 PIO ADC init failed");

  currentSense0.linkDriver(&driver0);
  currentSense1.linkDriver(&driver1);
  currentSense0.skip_align = true;
  currentSense1.skip_align = true;
  currentSense0.init();
  currentSense1.init();

  configureMotor(motor0, driver0, currentSense0);
  configureMotor(motor1, driver1, currentSense1);

  Serial.println("Open-loop velocity started");
}

void loop() {
  motor0.move(TARGET_VELOCITY);
  motor1.move(TARGET_VELOCITY);

  const uint32_t nowMs = millis();
  if ((nowMs - lastCurrentPrintMs) >= CURRENT_PRINT_INTERVAL_MS) {
    lastCurrentPrintMs = nowMs;
    printCurrents();
  }
}
