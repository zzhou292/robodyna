// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "GroupScreenValues.h"
#include "Screen.h"

namespace tl::fea::cin_advance::group_screen {
__device__ inline void ScreenGroup(const Input& input, std::uint32_t group, Tile& tile) {
  const auto source = screen::Sources(input);
  if (!threadIdx.x) tile.group = BeginGroup(source, group);
  __syncthreads();
  if (tile.group.stopped) {
    if (!threadIdx.x) input.group_reports[group] = tile.group.report;
    return;
  }
  const auto range = source.rigid.groups[group];
  const auto body = ReadBody(tile.group.body);
  for (std::uint32_t first = 0; first < range.count;) {
    const auto remaining = range.count-first;
    const auto count = remaining < Threads ? remaining : Threads;
    if (threadIdx.x < count)
      tile.members[threadIdx.x] = PrepareMember(source, body, range.offset+first+threadIdx.x);
    __syncthreads();
    if (!threadIdx.x) {
      for (unsigned local = 0; local < count && !tile.group.stopped; ++local)
        FoldMember(source, body, range.offset+first+local, tile.members[local], tile.group);
    }
    __syncthreads();
    if (tile.group.stopped) break; // Uniform: all threads observed the leader's write.
    first += count; // The bounded remainder also prevents a final uint32 wrap.
  }
  __syncthreads(); // End the shared stop-flag read phase before final publication.
  if (!threadIdx.x) {
    FinishGroup(input.structural.factor, tile.group);
    input.group_reports[group] = tile.group.report;
  }
}
} // namespace tl::fea::cin_advance::group_screen
