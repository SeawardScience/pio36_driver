#pragma once

#include "block.hpp"


BLOCKS_NS_HEAD

template <int  ADDR, int  NUM_WORDS>
class WordBuffer : public Block{
protected:
  primitives::word buffer_[NUM_WORDS];

public:
  WordBuffer(){
    for (int i = 0; i < NUM_WORDS; ++i) {
      buffer_[i]=0;
    }
  }
  primitives::WordBuffer buffer(){return buffer_;}
  size_t size(){return NUM_WORDS;}

  int innovaAddress(){return ADDR;}
};

BLOCKS_NS_FOOT
