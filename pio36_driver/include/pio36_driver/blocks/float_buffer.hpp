#pragma once

#include "block.hpp"


BLOCKS_NS_HEAD

/**
 * @brief Generic block of consecutive big-endian 32-bit floats
 * @tparam ADDR Innova register address (1-indexed)
 * @tparam NUM_FLOATS Number of floats (2 words each)
 */
template <int ADDR, int NUM_FLOATS>
class FloatBuffer : public Block{
protected:
  static const size_t BLOCK_SIZE = NUM_FLOATS * 2;
  union{
    primitives::BE_f32 values[NUM_FLOATS];
    primitives::word word_buffer[BLOCK_SIZE];
  } data_;

public:
  FloatBuffer(){
    for (size_t i = 0; i < BLOCK_SIZE; ++i) {
      data_.word_buffer[i]=0;
    }
  }
  primitives::WordBuffer buffer(){return data_.word_buffer;}
  size_t size(){return BLOCK_SIZE;}

  /** @brief Access float i (0-indexed) */
  primitives::BE_f32 & at(size_t i){return data_.values[i];}

  int innovaAddress(){return ADDR;}
};

BLOCKS_NS_FOOT
