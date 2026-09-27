// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../solvers/NodalTrialIdentity.h"

namespace tl::fea::shell_source_range_detail {
// The caller proves that every protected subobject lies between these endpoints
// of one immutable contiguous allocation. A disjoint envelope certifies that its
// exact per-record scan can be skipped. An overlap or malformed endpoint only
// means "scan normally": gaps and padding are not new rejected output ranges.
inline bool DisjointEnvelope(const void* output,std::size_t bytes,
    const void* first,std::size_t first_bytes,
    const void* last,std::size_t last_bytes) noexcept {
  const auto begin=reinterpret_cast<std::uintptr_t>(first);
  const auto tail=reinterpret_cast<std::uintptr_t>(last);
  if (!first || !last || !first_bytes || !last_bytes || tail<begin ||
      first_bytes>UINTPTR_MAX-begin || last_bytes>UINTPTR_MAX-tail) return false;
  const auto first_end=begin+first_bytes;
  const auto last_end=tail+last_bytes;
  const auto end=first_end>last_end?first_end:last_end;
  if (end-begin>SIZE_MAX) return false;
  return trial_identity::Disjoint(output,bytes,first,static_cast<std::size_t>(end-begin));
}
} // namespace tl::fea::shell_source_range_detail
