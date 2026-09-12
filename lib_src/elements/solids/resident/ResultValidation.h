// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "DeviceFamilies.h"
#include "ResultChecks.h"

namespace tl::fea::solids::batch_detail {
template<class Traits>
TL_BRICK_HD inline std::uint8_t CheckParentResult(Storage& state, unsigned trial,
    std::size_t parent_index, double time, std::uint64_t epoch) noexcept {
  const auto& family = FamilyStorage<Traits>(state);
  // A failed update may leave an unavailable trial. Preserve the original
  // status short-circuit before accessing its reference/material/history.
  if (family.status[parent_index] != 0) return 0;
  const auto& parent = family.parents[parent_index];
  return ValidResult(parent, MaterialAt<Traits>(state, parent.material_index),
      family.slab[trial][parent_index], time, epoch) ? 1 : 0;
}

template<class Traits> struct DirectResultCheck {
  Storage& state;
  unsigned trial;
  TL_BRICK_HD bool operator()(std::size_t parent, double time, std::uint64_t epoch) const noexcept {
    return CheckParentResult<Traits>(state, trial, parent, time, epoch) == 1;
  }
};
struct StagedResultCheck {
  const std::uint8_t* flags;
  TL_BRICK_HD bool operator()(std::size_t parent, double, std::uint64_t) const noexcept {
    return flags[parent] == 1;
  }
};
} // namespace tl::fea::solids::batch_detail
