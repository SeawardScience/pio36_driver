#pragma once

#include "block.hpp"

BLOCKS_NS_HEAD

/**
 * @brief PWM dither frequency register block (Reg 1074-1104, 16 channels)
 * 
 * Each channel stores dither frequency as 32-bit float in Hz.
 * Dither calculated on every PWM update - frequencies above Nyquist 
 * of PWM frequency won't work properly.
 */
class PwmDitherFrequency : public Block {
public:
  static const size_t NUM_PWM_CHANNELS = 16;
  
protected:
  struct PwmDitherFrequency_T {
    primitives::BE_f32 dither_frequency[NUM_PWM_CHANNELS];
  };
  static const size_t BLOCK_SIZE = sizeof(PwmDitherFrequency_T)/2;
  union {
    PwmDitherFrequency_T type;
    primitives::word word_buffer[BLOCK_SIZE];
  } data_;

public:
  std::shared_ptr<PwmDitherFrequency> SharedPtr;
  
  PwmDitherFrequency() {
    for (size_t i = 0; i < BLOCK_SIZE; ++i) {
      data_.word_buffer[i] = 0;
    }
  }
  
  primitives::WordBuffer buffer() { return data_.word_buffer; }
  size_t size() { return BLOCK_SIZE; }
  
  /**
   * @brief Access dither frequency for specific channel
   * @param chan Channel number (1-16, matches board labels)
   * @return Reference to dither frequency in Hz
   */
  primitives::BE_f32& ditherFreqChan(size_t chan) { 
    return data_.type.dither_frequency[chan-1]; 
  }
  
  int innovaAddress() { return 1074; }
};

BLOCKS_NS_FOOT
