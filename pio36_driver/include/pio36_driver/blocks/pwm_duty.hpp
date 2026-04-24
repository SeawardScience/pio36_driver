#pragma once

#include "block.hpp"


BLOCKS_NS_HEAD

/**
 * @brief PWM duty cycle register block (Reg 1-36, 16 channels)
 * 
 * Each channel stores duty cycle as 32-bit float (0-100%).
 * Channel numbering matches board labels (1-16, not 0-indexed).
 */
class PwmDuty : public Block{

public:
  static const size_t NUM_PWM_CHANNELS = 16;
protected:
  struct PwmDuty_T{
    primitives::BE_f32 duty[NUM_PWM_CHANNELS];
  };
  static const size_t BLOCK_SIZE = sizeof(PwmDuty_T)/2;
  union{
    PwmDuty_T type;
    primitives::word word_buffer[BLOCK_SIZE];
  } data_;

public:
  std::shared_ptr<PwmDuty> SharedPtr;
  PwmDuty(){
    for (size_t i = 0; i < BLOCK_SIZE; ++i) {
      data_.word_buffer[i]=0;
    }};
  primitives::WordBuffer buffer(){return data_.word_buffer;}
  size_t size(){return BLOCK_SIZE;}
  /*!
   * \brief dutyChan returns a reference to the duty at channel i
   * \param chan the duty cycle channel you want to access.   This
   * corresponds exacty to the number printed on the board (1-16).
   * \return a reference to the channel you want to access.   This
   * is a value from 0-100
   * \note This value is not indexed from zero!  It corresponds
   * to the channel printed on the board.
   */
  primitives::BE_f32 & dutyChan(size_t chan){return data_.type.duty[chan-1];}
  /*!
   * \brief innovaAddress returns the of the PWM block in the innova documentation
   * \return
   */
  int innovaAddress(){return 1000;}
};

BLOCKS_NS_FOOT
