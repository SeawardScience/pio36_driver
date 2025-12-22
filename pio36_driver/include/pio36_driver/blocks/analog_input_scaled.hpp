#pragma once

#include "block.hpp"


BLOCKS_NS_HEAD

/**
 * @brief Analog voltage input register block (Reg 37-44, 8 channels)
 * 
 * Each channel stores voltage as 32-bit float (0-10V range).
 * Channel numbering matches board labels (1-8, not 0-indexed).
 */
class AnalogVoltageInput : public Block{

public:
  static const size_t NUM_CHANNELS = 4;
protected:
  struct Analog_T{
    primitives::BE_f32 analog[NUM_CHANNELS];
  };
  static const size_t BLOCK_SIZE = sizeof(Analog_T)/2;
  union{
    Analog_T type;
    primitives::word word_buffer[BLOCK_SIZE];
  } data_;

public:
  std::shared_ptr<AnalogVoltageInput> SharedPtr;
  AnalogVoltageInput(){
    for (size_t i = 0; i < BLOCK_SIZE; ++i) {
      data_.word_buffer[i]=0;
    }};
  primitives::WordBuffer buffer(){return data_.word_buffer;}
  size_t size(){return BLOCK_SIZE;}

  /** @brief Access analog voltage channel (1-8) */
  primitives::BE_f32 & chanel(size_t chan){return data_.type.analog[chan-1];}
  /*!
   * \brief innovaAddress returns the of the PWM block in the innova documentation
   * \return
   */
  int innovaAddress(){return 2040;}
};

BLOCKS_NS_FOOT
