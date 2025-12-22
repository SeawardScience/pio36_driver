#pragma once
#include "../package_defs.hpp"
#include <cstdint>
#include <stddef.h>
NS_HEAD


namespace primitives {

typedef unsigned int   u32;
typedef signed char    s8;
typedef signed short   s16;
typedef signed int     s32;
typedef float          f32;
typedef double         f64;
typedef uint16_t       word;
typedef word*          WordBuffer;

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

typedef BigEndianPrimative<u32>     BE_u32;
typedef BigEndianPrimative<s8>      BE_s8;
typedef BigEndianPrimative<s16>     BE_s16;
typedef BigEndianPrimative<s32>     BE_s32;
typedef BigEndianPrimative<f32>     BE_f32;
typedef BigEndianPrimative<f64>     BE_f64;

}
NS_FOOT
