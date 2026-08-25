#pragma once

#include <stdint.h>
#include "hardware/spi.h"
#include "board_config.h"

static constexpr uint8_t SHARED_SPI_PROFILE_UNKNOWN = 0;
static constexpr uint8_t SHARED_SPI_PROFILE_ENCODER = 1;
static constexpr uint8_t SHARED_SPI_PROFILE_DRV8316 = 2;

extern uint8_t sharedSpiProfile;
extern bool sharedSpiInitialized;

static inline void sharedSpiMarkUnknown() {
  sharedSpiProfile = SHARED_SPI_PROFILE_UNKNOWN;
}

static inline void sharedSpiUseProfile(uint8_t profile, uint32_t baudrate) {
  if (!sharedSpiInitialized) {
    spi_init(spi0, baudrate);
    sharedSpiInitialized = true;
    sharedSpiProfile = SHARED_SPI_PROFILE_UNKNOWN;
  }

  if (sharedSpiProfile == profile) {
    return;
  }

  spi_set_baudrate(spi0, baudrate);
  spi_set_format(spi0, 16, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
  sharedSpiProfile = profile;
}

static inline void sharedSpiUseEncoder() {
  sharedSpiUseProfile(SHARED_SPI_PROFILE_ENCODER, ENCODER_SPI_HZ);
}

static inline void sharedSpiUseDrv8316() {
  sharedSpiUseProfile(SHARED_SPI_PROFILE_DRV8316, DRV8316_SPI_HZ);
}
