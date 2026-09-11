// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Arena.h"

namespace tl::fea::solids::batch_detail {
template<class Traits>
TL_BRICK_HD inline DeviceFamily<Traits>& FamilyStorage(Storage& state) noexcept {
  if constexpr (std::is_same_v<Traits, Traits18>) return state.solid18;
  else if constexpr (std::is_same_v<Traits, Traits24>) return state.solid24;
  else {
    static_assert(std::is_same_v<Traits, Traits6z>);
    return state.solid6z;
  }
}
template<class Traits>
TL_BRICK_HD inline const typename Traits::Material& MaterialAt(Storage& state,
    std::size_t index) noexcept {
  if constexpr (std::is_same_v<Traits, Traits18>) return state.material36[index];
  else return state.material42[index];
}
} // namespace tl::fea::solids::batch_detail
