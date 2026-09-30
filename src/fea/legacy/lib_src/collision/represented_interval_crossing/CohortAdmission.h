// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "BatchExecution.h"
namespace tlfea::contact::represented_interval_crossing::detail {
// Pure range proof, not a source-validity or cached-result authority. Ordinary
// compound validation has already checked all rows before this helper is used.
// False disables lookahead; it does not replace the original slice diagnostics.
template <class OwnedDisjoint, class RangeDisjoint>
bool NumericCohortRanges(const RepresentedTrianglePath* paths, std::size_t path_count,
    const RepresentedTrianglePair* pairs, std::size_t pair_count,
    const OwnedDisjoint& owned, const RangeDisjoint& disjoint) noexcept {
  std::size_t path_bytes = 0, pair_bytes = 0;
  return Bytes(paths, path_count, &path_bytes) && Bytes(pairs, pair_count, &pair_bytes) &&
      disjoint(paths, path_bytes, pairs, pair_bytes) &&
      owned(paths, path_bytes) && owned(pairs, pair_bytes);
}
}  // namespace tlfea::contact::represented_interval_crossing::detail
