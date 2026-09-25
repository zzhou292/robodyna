// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "HostDevice.h"
#include <type_traits>
namespace tl::math {
// Compare individual binary32/binary64 source values without padding, FP
// comparisons, aliasing violations, or host/device-specific type definitions.
template<class Scalar>
TL_MATH_HOST_DEVICE inline bool SameScalarBits(Scalar a, Scalar b) noexcept {
  static_assert(std::is_same_v<Scalar,float> || std::is_same_v<Scalar,double>);
  static_assert(sizeof(float)==4 && sizeof(double)==8);
  const auto* first=reinterpret_cast<const unsigned char*>(&a);
  const auto* second=reinterpret_cast<const unsigned char*>(&b);
  for(unsigned i=0;i<sizeof(Scalar);++i)if(first[i]!=second[i])return false;
  return true;
}
} // namespace tl::math
