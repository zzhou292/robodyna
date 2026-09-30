// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalNodeDomain.h"
#include <cstring>
#include <limits>

namespace tl::fea::nodal_domain_detail {
static_assert(sizeof(double)==8&&std::numeric_limits<double>::is_iec559);
inline bool SamePosition(tl::math::Vec3 a,tl::math::Vec3 b) noexcept {
  // Same named binary64 comparison as native model identities; never struct padding.
  return std::memcmp(&a.x,&b.x,sizeof(double))==0&&
      std::memcmp(&a.y,&b.y,sizeof(double))==0&&std::memcmp(&a.z,&b.z,sizeof(double))==0;
}
template<class T> bool ValidRange(const T* pointer,std::size_t count) noexcept {
  const auto address=reinterpret_cast<std::uintptr_t>(pointer);
  return pointer&&address%alignof(T)==0&&count<=(UINTPTR_MAX-address)/sizeof(T);
}
} // namespace tl::fea::nodal_domain_detail
