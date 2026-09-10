#include "BU79100QuadReader.h"

#include <cstring>
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "control_math.h"
#include "current/rp2350/bu79100g_parallel3.pio.h"

BU79100QuadReader *BU79100QuadReader::activeReader_ = nullptr;

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
  if (activeReader_ != nullptr) return false;
  sm_ = pio_claim_unused_sm(pio_, false);
  if (sm_ < 0) {
    return false;
  }

  uint16_t patchedInstructions[sizeof(bu79100g_parallel3_program_instructions) / sizeof(uint16_t)];
  memcpy(patchedInstructions, bu79100g_parallel3_program_instructions, sizeof(patchedInstructions));
  patchedInstructions[1] = (patchedInstructions[1] & ~0x1Fu) | (pinTrigger_ & 0x1Fu);
  patchedInstructions[2] = (patchedInstructions[2] & ~0x1Fu) | (pinTrigger_ & 0x1Fu);

  pio_program program = bu79100g_parallel3_program;
  program.instructions = patchedInstructions;

  if (!pio_can_add_program(pio_, &program)) {
    pio_sm_unclaim(pio_, sm_);
    return false;
  }
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

  dmaA_ = dma_claim_unused_channel(false);
  if (dmaA_ < 0) {
    pio_remove_program(pio_, &program, offset);
    pio_sm_unclaim(pio_, sm_);
    return false;
  }

  const uint dreqBase = (pio_ == pio0) ? DREQ_PIO0_RX0 : DREQ_PIO1_RX0;

  dma_channel_config configA = dma_channel_get_default_config(dmaA_);
  channel_config_set_read_increment(&configA, false);
  channel_config_set_write_increment(&configA, true);
  channel_config_set_transfer_data_size(&configA, DMA_SIZE_32);
  channel_config_set_dreq(&configA, dreqBase + sm_);

  channel_config_set_ring(&configA, true, __builtin_ctz(RING_BYTES));

  dma_channel_set_config(dmaA_, &configA, false);
  dma_channel_set_read_addr(dmaA_, &pio_->rxf[sm_], false);
  dma_channel_set_write_addr(dmaA_, (void*)buffer_, false);
  dma_channel_set_trans_count(dmaA_, RING_WORDS, false);

  // Reload once per 32 frames (625 IRQ/s), preserving a monotonic frame ID.
  // The reader and this IRQ run on the same core.
  activeReader_ = this;
  dma_channel_acknowledge_irq1(dmaA_);
  irq_add_shared_handler(DMA_IRQ_1, dmaInterrupt, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
  dma_channel_set_irq1_enabled(dmaA_, true);
  irq_set_enabled(DMA_IRQ_1, true);
  pio_->fdebug = 1u << (PIO_FDEBUG_RXSTALL_LSB + sm_);

  dma_channel_start(dmaA_);
  pio_sm_set_enabled(pio_, sm_, true);
  return true;
}

void BU79100QuadReader::dmaInterrupt() {
  auto *reader = activeReader_;
  if (!reader || !dma_channel_get_irq1_status(reader->dmaA_)) return;
  dma_channel_acknowledge_irq1(reader->dmaA_);
  reader->completedFrames_ += RING_WORDS / 2;
  reader->receivedBlock_ = true;
  dma_channel_set_trans_count(reader->dmaA_, RING_WORDS, true);
}

BU79100QuadSample BU79100QuadReader::read() const {
  BU79100QuadSample sample;
  if (dmaA_ < 0) return sample;

  // Prevent the reload IRQ from changing the epoch between these reads. DMA
  // itself continues; it only writes after the most recently completed pair.
  const uint32_t irq = save_and_disable_interrupts();
  const uint32_t words = RING_WORDS - dma_hw->ch[dmaA_].transfer_count;
  const uint32_t frames = words / 2;
  sample.sequence = completedFrames_ + frames;
  sample.valid = (receivedBlock_ || frames != 0) &&
    !(pio_->fdebug & (1u << (PIO_FDEBUG_RXSTALL_LSB + sm_))) &&
    !(dma_hw->ch[dmaA_].ctrl_trig & DMA_CH0_CTRL_TRIG_AHB_ERROR_BITS);
  if (sample.valid && cached_.valid && sample.sequence == cached_.sequence) {
    restore_interrupts(irq);
    return cached_;
  }
  if (sample.valid) {
    const uint32_t first = (frames * 2 + RING_WORDS - 2) & (RING_WORDS - 1);
    sample.word0 = buffer_[first];
    sample.word1 = buffer_[first + 1];
  }
  restore_interrupts(irq);
  if (sample.valid)
    control_math::decodeAdcWords(sample.word0, sample.word1, sample.raw);
  cached_ = sample;
  return sample;
}
