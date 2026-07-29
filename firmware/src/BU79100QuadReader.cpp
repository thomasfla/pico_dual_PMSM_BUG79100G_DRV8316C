#include "BU79100QuadReader.h"

#include <cstring>
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "current/rp2350/bu79100g_parallel3.pio.h"

BU79100QuadReader::BU79100QuadReader(
  PIO pio,
  uint8_t pinSck,
  uint8_t pinCsb,
  uint8_t pinD0,
  uint8_t pinTrigger
) : pio_(pio),
    pinSck_(pinSck),
    pinCsb_(pinCsb),
    pinD0_(pinD0),
    pinTrigger_(pinTrigger) {}

bool BU79100QuadReader::init(float sckHz) {
  sm_ = pio_claim_unused_sm(pio_, true);
  if (sm_ < 0) {
    return false;
  }

  uint16_t patchedInstructions[sizeof(bu79100g_parallel3_program_instructions) / sizeof(uint16_t)];
  memcpy(patchedInstructions, bu79100g_parallel3_program_instructions, sizeof(patchedInstructions));
  patchedInstructions[1] = (patchedInstructions[1] & ~0x1Fu) | (pinTrigger_ & 0x1Fu);
  patchedInstructions[2] = (patchedInstructions[2] & ~0x1Fu) | (pinTrigger_ & 0x1Fu);

  pio_program program = bu79100g_parallel3_program;
  program.instructions = patchedInstructions;

  const uint offset = pio_add_program(pio_, &program);
  pio_sm_config config = bu79100g_parallel3_program_get_default_config(offset);

  sm_config_set_in_pins(&config, pinD0_);
  sm_config_set_set_pins(&config, pinCsb_, 1);
  sm_config_set_sideset_pins(&config, pinSck_);
  sm_config_set_in_shift(&config, false, true, 32);

  const float divider = (float)clock_get_hz(clk_sys) / (2.0f * sckHz);
  sm_config_set_clkdiv(&config, divider);

  pio_gpio_init(pio_, pinSck_);
  pio_gpio_init(pio_, pinCsb_);
  for (uint8_t pin = pinD0_; pin < pinD0_ + 4; pin++) {
    pio_gpio_init(pio_, pin);
  }

  pio_sm_set_consecutive_pindirs(pio_, sm_, pinSck_, 1, true);
  pio_sm_set_consecutive_pindirs(pio_, sm_, pinCsb_, 1, true);
  pio_sm_set_consecutive_pindirs(pio_, sm_, pinD0_, 4, false);
  pio_sm_init(pio_, sm_, offset, &config);

  dmaA_ = dma_claim_unused_channel(true);
  dmaB_ = dma_claim_unused_channel(true);

  const uint dreqBase = (pio_ == pio0) ? DREQ_PIO0_RX0 : DREQ_PIO1_RX0;

  dma_channel_config configA = dma_channel_get_default_config(dmaA_);
  channel_config_set_read_increment(&configA, false);
  channel_config_set_write_increment(&configA, true);
  channel_config_set_transfer_data_size(&configA, DMA_SIZE_32);
  channel_config_set_dreq(&configA, dreqBase + sm_);
  channel_config_set_chain_to(&configA, dmaB_);
  channel_config_set_ring(&configA, true, __builtin_ctz(RING_BYTES));

  dma_channel_set_config(dmaA_, &configA, false);
  dma_channel_set_read_addr(dmaA_, &pio_->rxf[sm_], false);
  dma_channel_set_write_addr(dmaA_, (void*)buffer_, false);
  dma_channel_set_trans_count(dmaA_, RING_WORDS, false);

  dma_channel_config configB = dma_channel_get_default_config(dmaB_);
  channel_config_set_read_increment(&configB, false);
  channel_config_set_write_increment(&configB, false);
  channel_config_set_transfer_data_size(&configB, DMA_SIZE_32);
  dma_channel_configure(
    dmaB_,
    &configB,
    (void*)&dma_hw->ch[dmaA_].al1_transfer_count_trig,
    (const void*)&reloadCount_,
    1,
    false
  );

  dma_channel_start(dmaA_);
  pio_sm_set_enabled(pio_, sm_, true);
  return true;
}

BU79100QuadSample BU79100QuadReader::read() const {
  BU79100QuadSample sample;
  if (dmaA_ < 0) {
    return sample;
  }

  const uintptr_t base = (uintptr_t)buffer_;
  const uint32_t nextWrite =
    (((uintptr_t)dma_hw->ch[dmaA_].write_addr - base) >> 2) & (RING_WORDS - 1);
  const uint32_t firstWord = ((nextWrite & ~1u) + RING_WORDS - 2) & (RING_WORDS - 1);

  sample.word0 = buffer_[firstWord];
  sample.word1 = buffer_[(firstWord + 1) & (RING_WORDS - 1)];

  for (int shift = 12; shift >= 0; shift -= 4) {
    appendNibble((sample.word0 >> shift) & 0xFu, sample.raw);
  }
  for (int shift = 28; shift >= 0; shift -= 4) {
    appendNibble((sample.word1 >> shift) & 0xFu, sample.raw);
  }

  return sample;
}

void BU79100QuadReader::appendNibble(uint32_t nibble, uint16_t raw[4]) {
  raw[0] = (raw[0] << 1) | ((nibble >> 0) & 1u);
  raw[1] = (raw[1] << 1) | ((nibble >> 1) & 1u);
  raw[2] = (raw[2] << 1) | ((nibble >> 2) & 1u);
  raw[3] = (raw[3] << 1) | ((nibble >> 3) & 1u);
}
