// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include "lib_src/math/HostDevice.h"
namespace tlfea::contact::radioss_type25::assembly {
// Schedule has been host-validated and remains immutable during device work.
// Binary lookup retains native cohort boundaries without storing a second
// per-occurrence descriptor array. Ranks are at most5*row_count-1.
TL_MATH_HOST_DEVICE inline bool DecodeOccurrence(Schedule schedule,
    std::uint32_t rank, Occurrence* output) {
  if (!output || !schedule.cohort_ends || !schedule.cohort_count ||
      !schedule.row_count || schedule.row_count > UINT32_MAX / 5 ||
      rank >= 5 * schedule.row_count) return false;
  std::size_t lo = 0, hi = schedule.cohort_count;
  while (lo < hi) {
    const auto mid = lo + (hi - lo) / 2;
    if (rank < 5ull * schedule.cohort_ends[mid]) hi = mid;
    else lo = mid + 1;
  }
  if (lo == schedule.cohort_count) return false;
  const auto first = lo ? schedule.cohort_ends[lo - 1] : 0u;
  const auto end = schedule.cohort_ends[lo];
  if (end <= first || end > schedule.row_count || rank < 5ull * first) return false;
  const auto offset = rank - 5 * first;
  const auto main_count = 4 * (end - first);
  *output = offset < main_count ? Occurrence{first + offset / 4, offset % 4} :
      Occurrence{first + offset - main_count, 4};
  return output->row < end;
}
TL_MATH_HOST_DEVICE inline std::uint32_t EndpointNode(const Connectivity& row,
    unsigned slot) { return slot < 4 ? row.main[slot] : row.secondary; }
} // namespace tlfea::contact::radioss_type25::assembly
