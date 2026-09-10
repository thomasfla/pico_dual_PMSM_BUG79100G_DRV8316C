#pragma once

#include "drivers/drv8316/drv8316.h"

// This board uses GPIO 0..5 (three complete slices) and GPIO 27 for ADC timing.
// No SimpleFOC hardware backend or private driver parameter layout is used.
namespace board_pwm {
void init();
void start();
void beginFrame();
bool finishFrame(); // stage this frame's output for the next ADC frame
bool commitStagedFrame();
bool commitFrame(); // immediate commit, used by the inductance test
uint32_t lastFrameDurationUs();
void stop();
float periodUs();
float samplePhaseUs();
bool samplingWindow();
}

class BoardMotorDriver : public DRV8316Driver3PWM {
public:
  BoardMotorDriver(uint8_t motor, int cs, int fault)
    : DRV8316Driver3PWM(motor * 3, motor * 3 + 1, motor * 3 + 2,
                       cs, false, NOT_SET, fault), motor_(motor) {}
  void init(SPIClass *spi = &SPI) override;
  void setPwm(float ua, float ub, float uc) override;
  void updateSupplyVoltage(float supply);
private:
  uint8_t motor_;
  float countsPerVolt_ = 0.0f;
};
