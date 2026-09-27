// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../QbatBatchResultChecks.h"
namespace tl::fea::qbat::activity {
inline constexpr std::uint32_t NoInvalidParent = UINT32_MAX;
inline constexpr std::uint8_t InvalidFlag = 2;
// Same complete predicate as full ReadResults; only its execution location changes.
TL_QBAT_HD inline bool ValidParent(const batch_detail::Storage& state, unsigned slab,
    std::size_t parent, double time, std::uint64_t epoch) noexcept {
  return batch_detail::ValidResult(state.slab[slab].element[parent],
      state.model.element[parent].material, time, epoch);
}
inline BatchReport Invalid(std::uint32_t parent = UINT32_MAX) noexcept {
  return {BatchStatus::NonfiniteResult,"QBAT readback contains an invalid four-point result",parent};
}
// The catalog lookup is still performed first at each source parent. GPU failure
// and malformed transport flags join that same ordered decision, not an earlier
// global rejection. No caller output is written by this helper.
template<class Lookup>
BatchReport Complete(std::uint32_t first_invalid, const std::uint8_t* flags,
    std::size_t count, Lookup lookup) noexcept {
  if (first_invalid != NoInvalidParent && first_invalid >= count) return Invalid();
  for (std::size_t parent = 0; parent < count; ++parent) {
    if (!lookup(parent) || parent == first_invalid || flags[parent] > 1)
      return Invalid(static_cast<std::uint32_t>(parent));
  }
  return {BatchStatus::Success,"OK"};
}
} // namespace tl::fea::qbat::activity
