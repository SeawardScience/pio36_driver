#pragma once

#include "block.hpp"

BLOCKS_NS_HEAD

/**
 * @brief PWM dither duty register block (Reg 1042-1072, 16 channels)
 * 
 * Each channel stores dither amplitude as 32-bit float (0-100%).
 * Represents the amplitude of sine wave that modulates PWM duty.
 * Combined duty is clipped to 0-100% range.
 */
class PwmDitherDuty : public Block {
public:
  static const size_t NUM_PWM_CHANNELS = 16;
  
protected:
  struct PwmDitherDuty_T {
    primitives::BE_f32 dither_duty[NUM_PWM_CHANNELS];
  };
  static const size_t BLOCK_SIZE = sizeof(PwmDitherDuty_T)/2;
  union {
    PwmDitherDuty_T type;
    primitives::word word_buffer[BLOCK_SIZE];
  } data_;

public:
  std::shared_ptr<PwmDitherDuty> SharedPtr;
  
  PwmDitherDuty() {
    for (size_t i = 0; i < BLOCK_SIZE; ++i) {
      data_.word_buffer[i] = 0;
    }
  }
  
  primitives::WordBuffer buffer() { return data_.word_buffer; }
  size_t size() { return BLOCK_SIZE; }
  
  /**
   * @brief Access dither duty for specific channel
   * @param chan Channel number (1-16, matches board labels)
   * @return Reference to dither amplitude (0-100%)
   */
  primitives::BE_f32& ditherDutyChan(size_t chan) { 
    return data_.type.dither_duty[chan-1]; 
  }
  
  int innovaAddress() { return 1042; }
};

BLOCKS_NS_FOOT
