#pragma once

#include "block.hpp"

BLOCKS_NS_HEAD

/**
 * @brief PWM dither phase register block (Reg 1106-1136, 16 channels)
 * 
 * Each channel stores dither phase offset as 32-bit float in degrees (0-360).
 * Controls phase relationship between dither waveforms on different channels.
 */
class PwmDitherPhase : public Block {
public:
  static const size_t NUM_PWM_CHANNELS = 16;
  
protected:
  struct PwmDitherPhase_T {
    primitives::BE_f32 dither_phase[NUM_PWM_CHANNELS];
  };
  static const size_t BLOCK_SIZE = sizeof(PwmDitherPhase_T)/2;
  union {
    PwmDitherPhase_T type;
    primitives::word word_buffer[BLOCK_SIZE];
  } data_;

public:
  std::shared_ptr<PwmDitherPhase> SharedPtr;
  
  PwmDitherPhase() {
    for (size_t i = 0; i < BLOCK_SIZE; ++i) {
      data_.word_buffer[i] = 0;
    }
  }
  
  primitives::WordBuffer buffer() { return data_.word_buffer; }
  size_t size() { return BLOCK_SIZE; }
  
  /**
   * @brief Access dither phase for specific channel
   * @param chan Channel number (1-16, matches board labels)
   * @return Reference to dither phase in degrees (0-360)
   */
  primitives::BE_f32& ditherPhaseChan(size_t chan) { 
    return data_.type.dither_phase[chan-1]; 
  }
  
  int innovaAddress() { return 1106; }
};

BLOCKS_NS_FOOT
