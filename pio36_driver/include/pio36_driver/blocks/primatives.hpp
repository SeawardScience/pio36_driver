#pragma once
#include "../package_defs.hpp"
#include <cstdint>
#include <stddef.h>
NS_HEAD

/**
 * @brief Primitive types and big-endian conversion utilities for Modbus
 */
namespace primitives {

typedef unsigned int   u32;   ///< Unsigned 32-bit
typedef signed char    s8;    ///< Signed 8-bit
typedef signed short   s16;   ///< Signed 16-bit
typedef signed int     s32;   ///< Signed 32-bit
typedef float          f32;   ///< 32-bit float
typedef double         f64;   ///< 64-bit float
typedef uint16_t       word;  ///< Modbus 16-bit word
typedef word*          WordBuffer;  ///< Pointer to word buffer

template <typename T>
T revPrimitive(const T u)
{
    union
    {
        T u;
        uint16_t words[sizeof(T)];
    } source, dest;

    source.u = u;

    for (size_t k = 0; k < sizeof(T)/2; k++)
        dest.words[k] = source.words[sizeof(T) - k - 1];

    return dest.u;
}

/*!
 * \brief The BigEndianPrimative struct represents a big endian version
 * of various primitives.  Also includes overloaded casting operators
 * so it can be assigned to standard primatives with minimal extra syntax.
 */
template <typename PrimT>
struct BigEndianPrimative{
  union prim_union
  {
      PrimT u;
      uint16_t words[sizeof(PrimT)/2];
  } raw;
  /*!
   * \brief returns the machine-endian version of the variable in
   * the specified (PrimT) type.
   * \return a machine endian version fo the variable
   */
  PrimT get() const{
    prim_union output = raw;
    auto words = sizeof(PrimT)/2;
    for (size_t i = 0; i < words; i++){
      output.words[i] = raw.words[words - 1 - i];
    }
    return output.u;
  }
  void set(PrimT val){
    prim_union input;
    input.u = val;
    auto words = sizeof(PrimT)/2;
    for (size_t i = 0; i < words; i++){
      raw.words[i] = input.words[words - 1 - i];
    }
  }

  operator int8_t() const { return get(); }
  operator int16_t() const { return get(); }
  operator int32_t() const { return get(); }

  operator uint8_t() const { return get(); }
  operator uint16_t() const { return get(); }
  operator uint32_t() const { return get(); }

  operator float() const {return get(); }
  operator double() const {return get(); }

  template <typename T>
  BigEndianPrimative& operator=(T other){ this->set(other); return *this; }
};

/** @brief Big-endian unsigned 32-bit */
typedef BigEndianPrimative<u32>     BE_u32;
/** @brief Big-endian signed 8-bit */
typedef BigEndianPrimative<s8>      BE_s8;
/** @brief Big-endian signed 16-bit */
typedef BigEndianPrimative<s16>     BE_s16;
/** @brief Big-endian signed 32-bit */
typedef BigEndianPrimative<s32>     BE_s32;
/** @brief Big-endian 32-bit float */
typedef BigEndianPrimative<f32>     BE_f32;
/** @brief Big-endian 64-bit float */
typedef BigEndianPrimative<f64>     BE_f64;

}
NS_FOOT
